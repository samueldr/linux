/*
 * Copyright (C) 2002-2020 GoboLinux.org
 *
 * These modifications are released under the GNU General Public License
 * version 2 or later, incorporated herein by reference.
 * Modifications/features/bug fixes based on or derived from this code
 * fall under the GPL and must retain the authorship, copyright and license
 * notice.  This file is not a complete program and may only be used when
 * the entire operating system is licensed under the GPL.
 *
 * See the file COPYING in this distribution for more information.
 *
 * Author: Felipe W Damasio <felipewd@gmail.com>.
 * Original idea: Lucas C. Villa Real <lucasvr@gobolinux.org>
 *
 * Changes:
 * 12-Sep-2011 - Lucas C. Villa Real
 *               Take the superblock into account when comparing the dentries
 *               in order to allow mount points to be hidden.
 *
 * 03-Sep-2011 - Lucas C. Villa Real
 *               Security updates. Thanks to Dan Rosenberg for his code review.
 *
 * 18-May-2007 - Lucas C. Villa Real
 *               Added support for unionfs.
 *
 * 04-Jul-2006 - Lucas C. Villa Real
 *               Added GoboHide support for all filesystems through the VFS.
 *
 * 21-Feb-2004 - Lucas C. Villa Real
 *               Added an extra check for the inode's VFS root, so that
 *               the same inode number on different partitions don't get
 *               hidden mistakenly.
 *
 * 11-Nov-2003 - Lucas C. Villa Real
 *               Removed the spinlocks from gobolinux_show_hidden(), since
 *               we were already working with list_for_each_safe(), which
 *               iterates safely against removal of list entries.
 *
 * 05-May-2003 - Felipe W Damasio
 *               Using read-write locks instead of spinlocks,
 *               improving quite a bit read operations
 *               (allow concurrent readers, but only a single writer)
 *
 * 28-Apr-2003 - Lucas C. Villa Real
 *               Centralized checks for UID on gobolinux/fs/ioctl.c.
 *               Fixed get_free_page() to work on 64-bit archs as well.
 *
 * 12-Apr-2003 - Lucas C. Villa Real
 *               Disallow UID's different than 0 to hide inodes.
 *
 * 24-Mar-2003 - Lucas C. Villa Real
 *               Modified struct hide and calls so we have pathnames related
 *               to the "real" root dir and not the mount point.
 *
 * 17-Mar-2003 - Lucas C. Villa Real
 *               Added support for full pathname, rather than dealing only
 *               with inode numbers.
 *
 * 10-Jan-2003 - Lucas C. Villa Real
 *               Added statistics.
 */
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gobohide.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/mount.h>
#include <linux/path.h>
#include <linux/magic.h>
#include <net/genetlink.h>
#include <asm/uaccess.h>
#include "overlayfs/ovl_entry.h"
#include "mount.h"

static int gobohide_inode_list_count;
static LIST_HEAD(gobohide_inode_list);
static DEFINE_RWLOCK(gobohide_inode_rwlock);

static int gobohide_add(ino_t ino, const char *pathname);
static int gobohide_del(ino_t ino, const char *pathname);
static int gobohide_flush(void);
static struct hide **gobohide_list(u32 *count);
static int gobohide_remove_unlocked(struct hide *entry, int remove);
static struct hide *gobohide_get_unlocked(ino_t i_ino, const char *filename,
	int namelen, struct dentry *parent);

/* Netlink callbacks */

static int send_list_reply(struct genl_info *info, char *data, size_t size);
static int send_list_size_reply(struct genl_info *info, u32 size);

static int parse_path(struct nlattr *na, char **pathname)
{
	char *data;
	int len;

	if (na == NULL)
		return 1;
	len = nla_len(na);
	if (len > PATH_MAX)
		return -E2BIG;
	if (len < 1)
		return -EINVAL;
	data = kmalloc(len, GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	nla_strscpy(data, na, len);
	*pathname = data;
	return 0;
}

static int parse_attrs(struct genl_info *info, char **pathname, u64 *ino)
{
	int ret;

	if (!info->attrs[GOBOHIDE_CMD_ATTR_INODE] || !info->attrs[GOBOHIDE_CMD_ATTR_PATH])
		return -EINVAL;

	ret = parse_path(info->attrs[GOBOHIDE_CMD_ATTR_PATH], pathname);
	if (ret != 0)
		return ret;

	*ino = nla_get_u64(info->attrs[GOBOHIDE_CMD_ATTR_INODE]);
	return 0;
}

static int gobohide_add_cmd(struct sk_buff *skb, struct genl_info *info)
{
	char *pathname = NULL;
	u64 ino = 0;
	int ret;
	
	ret = parse_attrs(info, &pathname, &ino);
	if (ret < 0)
		return ret;
	ret = gobohide_add((ino_t) ino, pathname);
	kfree(pathname);
	return ret;
}

static int gobohide_del_cmd(struct sk_buff *skb, struct genl_info *info)
{
	char *pathname = NULL;
	u64 ino = 0;
	int ret;

	ret = parse_attrs(info, &pathname, &ino);
	if (ret < 0)
		return ret;
	ret = gobohide_del((ino_t) ino, pathname);
	kfree(pathname);
	return ret;
}

static int gobohide_flush_cmd(struct sk_buff *skb, struct genl_info *info)
{
	return gobohide_flush();
}

static int gobohide_list_cmd(struct sk_buff *skb, struct genl_info *info)
{
	struct hide **hiddenlist;
	int i = 0, ret = 0;
	u32 count = 0;

	hiddenlist = gobohide_list(&count);
	if (! hiddenlist)
		return 0;

	ret = send_list_size_reply(info, count);
	if (ret < 0)
		goto out_error;

	for (i=0; hiddenlist[i] != NULL; ++i) {
		struct hide *entry = hiddenlist[i];
		size_t size = nla_total_size(strlen(entry->pathname));

		ret = send_list_reply(info, entry->pathname, size);
		if (ret < 0)
			goto out_error;
		gobohide_put(entry);
	}

	kfree(hiddenlist);
	return ret;

out_error:
	while (hiddenlist[i] != NULL) {
		gobohide_put(hiddenlist[i]);
		i++;
	}
	kfree(hiddenlist);
	return ret;
}

/* Netlink interface registration */

static const struct nla_policy gobohide_nl_policy[GOBOHIDE_CMD_ATTR_MAX+1] = {
	[GOBOHIDE_CMD_ATTR_PATH] = { .type = NLA_STRING },
	[GOBOHIDE_CMD_ATTR_INODE] = { .type = NLA_U64 },
};

static struct genl_ops gobohide_nl_ops[] = {
	{
		.cmd	= GOBOHIDE_CMD_HIDE,
		.doit	= gobohide_add_cmd,
		.flags  = GENL_ADMIN_PERM,
	},
	{
		.cmd	= GOBOHIDE_CMD_UNHIDE,
		.doit	= gobohide_del_cmd,
		.flags  = GENL_ADMIN_PERM,
	},
	{
		.cmd	= GOBOHIDE_CMD_FLUSH,
		.doit	= gobohide_flush_cmd,
		.flags  = GENL_ADMIN_PERM,
	},
	{
		.cmd	= GOBOHIDE_CMD_LIST,
		.doit	= gobohide_list_cmd,
		.flags  = GENL_ADMIN_PERM,
	}
};

static struct genl_family family __ro_after_init = {
	.name    = GOBOHIDE_GENL_NAME,
	.version = GOBOHIDE_GENL_VERSION,
	.maxattr = GOBOHIDE_CMD_ATTR_MAX,
	.hdrsize = 0,
	.netnsok = true,
	.ops     = gobohide_nl_ops,
	.policy  = gobohide_nl_policy,
	.n_ops   = ARRAY_SIZE(gobohide_nl_ops),
};

static int send_list_size_reply(struct genl_info *info, u32 size)
{
	struct genlmsghdr *genlhdr;
	struct sk_buff *skb;
	void *reply;
	int ret;

	skb = genlmsg_new(size, GFP_KERNEL);
	if (! skb)
		return -ENOMEM;

	/* reply: list size as u32 */
	reply = genlmsg_put_reply(skb, info, &family, 0, GOBOHIDE_CMD_LIST_SIZE);
	if (! reply) {
		nlmsg_free(skb);
		return -EINVAL;
	}
	ret = nla_put(skb, GOBOHIDE_TYPE_LIST_SIZE, sizeof(u32), &size);
	if (ret < 0) {
		nlmsg_free(skb);
		return -EMSGSIZE;
	}

	/* send reply to listener */
	genlhdr = nlmsg_data(nlmsg_hdr(skb));
	genlmsg_end(skb, reply);
	return genlmsg_reply(skb, info);
}

static int send_list_reply(struct genl_info *info, char *data, size_t size)
{
	struct genlmsghdr *genlhdr;
	struct sk_buff *skb;
	struct nlattr *na;
	void *reply;

	skb = genlmsg_new(size, GFP_KERNEL);
	if (! skb)
		return -ENOMEM;

	/* reply: path as a string */
	reply = genlmsg_put_reply(skb, info, &family, 0, GOBOHIDE_CMD_LIST_REPLY);
	if (! reply) {
		nlmsg_free(skb);
		return -EINVAL;
	}
	na = nla_reserve(skb, GOBOHIDE_TYPE_PATH, size);
	if (! na) {
		nlmsg_free(skb);
		return -EMSGSIZE;
	}
	strncpy(nla_data(na), data, size);

	/* send reply to listener */
	genlhdr = nlmsg_data(nlmsg_hdr(skb));
	genlmsg_end(skb, reply);
	return genlmsg_reply(skb, info);
}

static int __init gobohide_netlink_init(void)
{
	return genl_register_family(&family);
}

static void __exit gobohide_netlink_exit(void)
{
	genl_unregister_family(&family);
}

late_initcall(gobohide_netlink_init);

/**
 * gobohide_resolve_path - Resolves the pathname of a given dentry
 * @entry: structure holding the dentry structure and the destination buffer.
 *
 * After the structure has been used, its member 'page' must be returned with
 * a call to free_page().
 */
static int gobohide_resolve_path(struct hide *entry)
{
	int len, ret;
	struct file *filp = entry->filp;
	struct path path = { .mnt = filp->f_path.mnt, .dentry = filp->f_path.dentry };

	entry->page = __get_free_page(GFP_USER);
	if (! entry->page)
		return -ENOMEM;

	entry->pathname = d_path(&path, (char *) entry->page, PAGE_SIZE);
	if (IS_ERR(entry->pathname)) {
		ret = PTR_ERR(entry->pathname);
		entry->pathname = NULL;
		free_page(entry->page);
		entry->page = 0;
		return ret;
	}

	len = PAGE_SIZE + entry->page - (unsigned long) entry->pathname;
	return len < PATH_MAX ? len : PATH_MAX;
}

/**
 * gobohide_list - Lists the currently hidden inodes.
 * @num[out]: number of entries returned
 *
 * @hide: output structure. We fill in its pathname_len and pathname members.
 */
static struct hide **gobohide_list(u32 *num)
{
	struct hide *entry, *next, **hiddenlist;
	int count, copied_entries = 0;
	unsigned long flags;

	read_lock(&gobohide_inode_rwlock);
	count = gobohide_inode_list_count;
	read_unlock(&gobohide_inode_rwlock);

	hiddenlist = kmalloc(sizeof(struct hide *) * count+1, GFP_KERNEL);
	if (! hiddenlist) {
		*num = 0;
		return NULL;
	}

	/* Since copy_to_user may sleep data can't be copied with the lock held */
	write_lock_irqsave(&gobohide_inode_rwlock, flags);
	if (gobohide_inode_list_count) {
		list_for_each_entry_safe(entry, next, &gobohide_inode_list, head) {
			if (entry && copied_entries < count) {
				/* Don't let the entry go away */
				entry->refcount++;
				hiddenlist[copied_entries++] = entry;
			} else
				break;
		}
	}
	hiddenlist[copied_entries] = NULL;
	write_unlock_irqrestore(&gobohide_inode_rwlock, flags);

	*num = count;
	return hiddenlist;
}

/* From overlayfs/util.c */
struct dentry *overlay_dentry_lower(struct dentry *dentry)
{
	struct ovl_entry *oe = dentry->d_fsdata;
	return oe->numlower ? oe->lowerstack[0].dentry : NULL;
}

/**
 * gobohide_add - Add the inode to the "must hide" list
 * @ino: inode to be added
 * @pathname: the pathname associated with @ino
 */
static int gobohide_add(ino_t ino, const char *pathname)
{
	int len, ret;
	struct path *path;
	struct hide *entry, *old;
	struct dentry *dentry, *lower = NULL;
	unsigned long flags;

	entry = kmalloc(sizeof(struct hide), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->refcount = 1;
	entry->unlinked = 0;

	path = &entry->path;
	ret = kern_path(pathname, 0, path);
	if (ret)
		goto out_free;

	entry->filp = dentry_open(path, O_NOFOLLOW|O_PATH, current_cred());
	if (IS_ERR(entry->filp)) {
		ret = PTR_ERR(entry->filp);
		goto out_path_put;
	}
	len = gobohide_resolve_path(entry);
	if (len < 0) {
		ret = len;
		goto out_fput;
	}
	dentry = file_dentry(entry->filp);
	if (! dentry) {
		ret = -ENOENT;
		goto out_free_page;
	}
	if (dentry->d_sb->s_magic == OVERLAYFS_SUPER_MAGIC) {
		lower = overlay_dentry_lower(dentry);
		entry->i_ino = d_inode(lower)->i_ino;
	} else
		entry->i_ino = d_inode(dentry)->i_ino;

	pr_debug("gobohide_add(%ld): real_ino=%ld lower=%ld\n",
			ino, entry->i_ino, lower ? d_inode(lower)->i_ino : 0);

	write_lock_irqsave(&gobohide_inode_rwlock, flags);
	old = gobohide_get_unlocked(entry->i_ino, dentry->d_name.name,
		dentry->d_name.len+1, dentry->d_parent);

	if (old) {
		write_unlock_irqrestore(&gobohide_inode_rwlock, flags);
		ret = -EEXIST;
		goto out_free_page;
	}

	gobohide_inode_list_count++;
	list_add(&entry->head, &gobohide_inode_list);
	write_unlock_irqrestore(&gobohide_inode_rwlock, flags);

	return 0;

out_free_page:
	free_page(entry->page);
out_fput:
	fput(entry->filp);
out_path_put:
	path_put(path);
out_free:
	kfree(entry);
	return ret;
}

/**
 * gobohide_flush - Removes all entries from the "must hide" list
 */
static int gobohide_flush(void)
{
	struct hide *entry, *aux;
	unsigned long flags;

	write_lock_irqsave(&gobohide_inode_rwlock, flags);
	if (gobohide_inode_list_count) {
		list_for_each_entry_safe(entry, aux, &gobohide_inode_list, head)
			gobohide_remove_unlocked(entry, 1);
	}
	write_unlock_irqrestore(&gobohide_inode_rwlock, flags);

	return 0;
}

/**
 * gobohide_del - Remove the inode from the "must hide" list
 * @ino: inode to be removed
 * @pathname: pathname to be removed
 */
static int gobohide_del(ino_t ino, const char *pathname)
{
	int len, ret;
	struct path *path;
	unsigned long flags;
	struct hide n, *entry, *aux;

	path = &n.path;
	ret = kern_path(pathname, 0, path);
	if (ret)
		return ret;

	n.filp = dentry_open(path, O_NOFOLLOW|O_PATH, current_cred());
	if (IS_ERR(n.filp)) {
		ret = PTR_ERR(n.filp);
		goto out_path_put;
	}
	len = gobohide_resolve_path(&n);
	if (len < 0) {
		ret = len;
		goto out_fput;
	}
	ino = file_inode(n.filp)->i_ino;

	ret = -ENOENT;
	write_lock_irqsave(&gobohide_inode_rwlock, flags);
	if (gobohide_inode_list_count) {
		list_for_each_entry_safe(entry, aux, &gobohide_inode_list, head) {
			struct dentry *filp_dentry = file_dentry(entry->filp);
			struct dentry *mnt_dentry =
				real_mount(entry->filp->f_path.mnt)->mnt_mountpoint;
			ino_t mnt_ino = d_inode(mnt_dentry)->i_ino;

			if ((entry->i_ino == ino && path->dentry->d_sb == filp_dentry->d_sb) ||
				(mnt_ino == ino && path->dentry->d_sb == mnt_dentry->d_sb)) {
				gobohide_remove_unlocked(entry, 1);
				ret = 0;
				break;
			}
		}
	}
	write_unlock_irqrestore(&gobohide_inode_rwlock, flags);

	free_page(n.page);
out_fput:
	fput(n.filp);
out_path_put:
	path_put(path);
	return ret;
}

/**
 * gobohide_remove - Effectively removes the inode from the inode_list.
 * @hide: struct hide to be removed
 */
int gobohide_remove(struct hide *entry)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&gobohide_inode_rwlock, flags);
	ret = gobohide_remove_unlocked(entry, 1);
	write_unlock_irqrestore(&gobohide_inode_rwlock, flags);

	return ret;
}

static int gobohide_remove_unlocked(struct hide *entry, int remove)
{
	if (remove && ! entry->unlinked) {
		/* Remove from the linked list */
		entry->unlinked = true;
		list_del(&entry->head);
		gobohide_inode_list_count--;
	}
	if (--entry->refcount == 0) {
		free_page(entry->page);
		fput(entry->filp);
		path_put(&entry->path);
		kfree(entry);
	}
	return 0;
}

/**
 * gobohide_get - Get the struct hide associated to the given inode. The inode
 *  is verified to exist in the "must hide" list through the comparison of the
 *  inode number and the superblock.
 *
 * @ino: inode being readdir'd
 * @filename: inode's filename
 * @namelen: inodes's filename length in bytes
 * @parent: the parent dentry for the given inode.
 *
 * If the inode number is in the inode_list, returns a pointer to its entry
 * in the inode_list or NULL if it isn't there. The returned entry must be
 * released with gobohide_put().
 */
struct hide *gobohide_get(ino_t ino, const char *filename, int namelen,
	struct dentry *parent)
{
	unsigned long flags;
	struct hide *entry;

	write_lock_irqsave(&gobohide_inode_rwlock, flags);
	entry = gobohide_get_unlocked(ino, filename, namelen, parent);
	write_unlock_irqrestore(&gobohide_inode_rwlock, flags);

	return entry;
}

static struct hide *gobohide_get_unlocked(ino_t ino, const char *filename,
	int namelen, struct dentry *parent)
{
	struct hide *entry = NULL;

	if (! ino || ! gobohide_inode_list_count)
		return NULL;

	pr_debug("gobohide_get(%ld|%.*s|%s)\n",
		ino, namelen, filename, parent->d_sb->s_id);

	list_for_each_entry(entry, &gobohide_inode_list, head) {
		struct dentry *filp_dentry = file_dentry(entry->filp);
		struct dentry *mnt_dentry =
			real_mount(entry->filp->f_path.mnt)->mnt_mountpoint;
		ino_t mnt_ino = d_inode(mnt_dentry)->i_ino;

		if ((entry->i_ino == ino && parent->d_sb == filp_dentry->d_sb) || 
			(mnt_ino == ino && parent->d_sb == mnt_dentry->d_sb)) {
			/* Increment the reference count and return the object */
			entry->refcount++;
			return entry;
		}
	}

	return NULL;
}

/*
 * Return an entry obtained from the gobohide_inode_list with gobohide_get().
 * @param entry Entry obtained from gobohide_get().
 */
int gobohide_put(struct hide *entry)
{
	unsigned long flags;
	int ret = -EINVAL;

	if (entry) {
		write_lock_irqsave(&gobohide_inode_rwlock, flags);
		ret = gobohide_remove_unlocked(entry, 0);
		write_unlock_irqrestore(&gobohide_inode_rwlock, flags);
	}

	return ret;
}

EXPORT_SYMBOL(gobohide_get);
EXPORT_SYMBOL(gobohide_put);
EXPORT_SYMBOL(gobohide_remove);
