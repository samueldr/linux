#ifndef _LINUX_GOBOHIDE_H
#define _LINUX_GOBOHIDE_H

#define GOBOHIDE_GENL_NAME    "gobohide"
#define GOBOHIDE_GENL_VERSION  0x01

/* netlink commands */
enum {
	GOBOHIDE_CMD_INVALID = 0,
	GOBOHIDE_CMD_HIDE,       /* userspace -> kernel */
	GOBOHIDE_CMD_UNHIDE,     /* userspace -> kernel */
	GOBOHIDE_CMD_FLUSH,      /* userspace -> kernel */
	GOBOHIDE_CMD_LIST,       /* userspace -> kernel */
	GOBOHIDE_CMD_LIST_SIZE,  /* kernel -> userspace */
	GOBOHIDE_CMD_LIST_REPLY, /* kernel -> userspace */
	__GOBOHIDE_CMD_MAX
};
#define GOBOHIDE_CMD_MAX (__GOBOHIDE_CMD_MAX - 1)

/* netlink policies */
enum {
	GOBOHIDE_CMD_ATTR_UNSPEC = 0,
	GOBOHIDE_CMD_ATTR_PATH,
	GOBOHIDE_CMD_ATTR_INODE,
	__GOBOHIDE_CMD_ATTR_MAX,
};
#define GOBOHIDE_CMD_ATTR_MAX (__GOBOHIDE_CMD_ATTR_MAX - 1)

/* netlink data types (kernel -> userspace) */
enum {
	GOBOHIDE_TYPE_UNSPECT = 0,
	GOBOHIDE_TYPE_PATH,
	GOBOHIDE_TYPE_LIST_SIZE,
	__GOBOHIDE_TYPE_MAX,
};
#define GOBOHIDE_TYPE_MAX (__GOBOHIDE_TYPE_MAX - 1)


#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/dcache.h>

/* internal structure representing a hidden entry */
struct hide {
   ino_t i_ino;            /* shortcut to inode number */
   struct file *filp;      /* used to recover the inode's pathname */
   struct path path;       /* stores the path after a call to user_lpath */
   char *pathname;         /* a fresh cache of the inode's pathname */
   unsigned long page;     /* page on which pathname has been copied to */
   unsigned long refcount; /* number of reference counts to this object */
   int unlinked;           /* has the structure been unlinked yet? */
   struct list_head head;  /* a simple doubly linked list */
};

#ifdef CONFIG_GOBOHIDE_FS
struct hide *gobohide_get(ino_t ino, const char *filename,
	int namelen, struct dentry *parent);
int  gobohide_put(struct hide *entry);
int  gobohide_remove(struct hide *hide);
#else
# define gobohide_get(ino, filename, namelen, parent) NULL
# define gobohide_put(entry) 0
# define gobohide_remove(hide) 0
#endif  /* CONFIG_GOBOHIDE_FS */
#endif  /* __KERNEL__ */
#endif  /* _LINUX_GOBOHIDE_H */
