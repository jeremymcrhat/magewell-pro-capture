

#include "common.h"


struct kmem_cache *g_spin_cache = NULL;
struct kmem_cache *g_mutex_cache = NULL;
struct kmem_cache *g_event_cache = NULL;



/* init */
int os_init(void)
{
    int ret;
    size_t size;

    /* kmem_cache_create require size >= BYTES_PER_WORD && size <= KMALLOC_MAX_SIZE */
    size = sizeof(spinlock_t) < sizeof(void *) ? sizeof(void *) : sizeof(spinlock_t);
    g_spin_cache = kmem_cache_create("mw-spin", size,
                                     0, 0, NULL);
    if (g_spin_cache == NULL)
        return -ENOMEM;

    g_mutex_cache = kmem_cache_create("mw-mutex", sizeof(struct mutex),
                                     0, 0, NULL);
    if (g_mutex_cache == NULL) {
        ret = -ENOMEM;
        goto mutex_err;
    }

    g_event_cache = kmem_cache_create("mw-event", sizeof(struct _os_event_t),
                                     0, 0, NULL);
    if (g_event_cache == NULL) {
        ret = -ENOMEM;
        goto event_err;
    }

    return 0;

event_err:
    kmem_cache_destroy(g_mutex_cache);
mutex_err:
    kmem_cache_destroy(g_spin_cache);
    return ret;
}


void os_deinit(void)
{
    if (g_spin_cache != NULL)
        kmem_cache_destroy(g_spin_cache);
    if (g_mutex_cache != NULL)
        kmem_cache_destroy(g_mutex_cache);
    if (g_event_cache != NULL)
        kmem_cache_destroy(g_event_cache);

    g_spin_cache = NULL;
    g_mutex_cache = NULL;
    g_event_cache = NULL;
}


int resource_pool_create(struct resource_pool *pool, int num_resources)
{
    resource_pool_priv *priv;
    int i;
    int ret = 0;

    if (NULL == pool || NULL != pool->priv)
        return -EINVAL;

    priv = os_malloc(sizeof(resource_pool_priv));
    if (NULL == priv)
        return -ENOMEM;

    priv->resources = os_malloc(sizeof(resource_item) * num_resources);
    if (NULL == priv->resources) {
        ret = -ENOMEM;
        goto res_err;
    }

    priv->lock = os_spin_lock_alloc();
    if (priv->lock == NULL) {
        ret = -ENOMEM;
        goto spin_err;
    }
    priv->wait = os_event_alloc();
    if (priv->wait == NULL) {
        ret = -ENOMEM;
        goto event_err;
    }

    INIT_LIST_HEAD(&priv->list_free);

    for (i=0; i < num_resources; i++) {
        priv->resources[i].index = i;
        list_add_tail(&(priv->resources[i].list_node), &priv->list_free);
    }

    priv->num_resources = num_resources;

    pool->priv = priv;

    return 0;

event_err:
    os_spin_lock_free(priv->lock);
spin_err:
    os_free(priv->resources);
res_err:
    os_free(priv);
    return ret;
}



void resource_pool_destroy(struct resource_pool *pool)
{
    resource_pool_priv *priv;

    if (NULL == pool || NULL == pool->priv)
        return;

    priv = pool->priv;

    if (priv->lock != NULL) {
        os_spin_lock_free(priv->lock);
        priv->lock = NULL;
    }

    if (priv->wait != NULL) {
        os_event_free(priv->wait);
        priv->wait = NULL;
    }

    if (NULL != priv->resources) {
        os_free(priv->resources);
        priv->resources = NULL;
    }

    os_free(priv);

    pool->priv = NULL;
}

int resource_pool_alloc_resource(struct resource_pool *pool)
{
    resource_pool_priv *priv = pool->priv;
    resource_item *pitem = NULL;
    int ret;

    do {
        os_spin_lock(priv->lock);
        if (!list_empty(&priv->list_free)) {
            pitem = list_entry(priv->list_free.next, resource_item, list_node);
            list_del(&pitem->list_node);
        }
        os_spin_unlock(priv->lock);

        if (NULL != pitem)
            return pitem->index;

        // wait
        ret = os_event_wait(priv->wait, -1);
        if (ret <= 0)
            return -EFAULT;
    } while (NULL == pitem);

    return -EFAULT;
}



void resource_pool_free_resource(struct resource_pool *pool, int index)
{
    resource_pool_priv *priv = pool->priv;
    resource_item *pitem = NULL;

    if (index < 0 || index >= priv->num_resources)
        return;

    pitem = &(priv->resources[index]);

    os_spin_lock(priv->lock);
    list_add_tail(&pitem->list_node, &priv->list_free);
    os_spin_unlock(priv->lock);

    os_event_set(priv->wait);
}


void *os_malloc(size_t size)
{
    struct _mhead * mem = NULL;
    size_t          memsize = sizeof (*mem) + size;
    unsigned char   mtype;
    unsigned long   kmalloc_max_size;

    if (size == 0) {
        return (0);
    }

#ifdef KMALLOC_MAX_CACHE_SIZE
    kmalloc_max_size = KMALLOC_MAX_CACHE_SIZE;
#else
    kmalloc_max_size = PAGE_SIZE;
#endif

    if (memsize <= kmalloc_max_size) {
        mem = (struct _mhead *)kmalloc(memsize, GFP_KERNEL);
        mtype = OS_LINUX_MEM_TYPE_KMALLOC;
    }

    if (mem == NULL) {
        memsize = PAGE_ALIGN(memsize);
        mem = (struct _mhead *)vmalloc(memsize);
        mtype = OS_LINUX_MEM_TYPE_VMALLOC;
    }

    if (!mem) {
        return (0);
    }

    mem->mtype = mtype;
    mem->msize = memsize;

    return mem->dat;
}



void *os_zalloc(size_t size)
{
    struct _mhead * mem = NULL;
    size_t          memsize = sizeof (*mem) + size;
    unsigned char   mtype;
    unsigned long   kmalloc_max_size;

    if (size == 0) {
        return (0);
    }

#ifdef KMALLOC_MAX_CACHE_SIZE
    kmalloc_max_size = KMALLOC_MAX_CACHE_SIZE;
#else
    kmalloc_max_size = PAGE_SIZE;
#endif

    if (memsize <= kmalloc_max_size) {
        mem = (struct _mhead *)kzalloc(memsize, GFP_KERNEL);
        mtype = OS_LINUX_MEM_TYPE_KMALLOC;
    }

    if (mem == NULL) {
        memsize = PAGE_ALIGN(memsize);
        mem = (struct _mhead *)vmalloc(memsize);
        if (mem != NULL) {
            memset(mem, 0, memsize);
        }
        mtype = OS_LINUX_MEM_TYPE_VMALLOC;
    }

    if (!mem) {
        return (0);
    }

    mem->mtype = mtype;
    mem->msize = memsize;

    return mem->dat;
}



void os_free(void *address)
{
    struct _mhead * hdr;

    hdr = (struct _mhead *)address;
    hdr--;

    if (hdr->mtype == OS_LINUX_MEM_TYPE_KMALLOC)
        kfree(hdr);
    else
        vfree(hdr);
}


os_contig_dma_desc_t os_contig_dma_alloc(size_t size, void *par)
{
    int nr_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
    void *parent = par;
    os_contig_dma_desc_t desc;

    desc = os_zalloc(sizeof(*desc));
    if (desc == NULL)
        return NULL;

    desc->size = nr_pages << PAGE_SHIFT;
    desc->virt_addr = dma_alloc_coherent(parent, desc->size, &desc->phys_addr, GFP_KERNEL);
    if (NULL == desc->virt_addr) {
        os_free(desc);
        return NULL;
    }

    desc->parent = parent;

    return desc;
}


void os_contig_dma_free(os_contig_dma_desc_t desc)
{
    if (desc == NULL)
        return;

    if (desc->virt_addr != NULL)
        dma_free_coherent(desc->parent, desc->size, desc->virt_addr, desc->phys_addr);

    os_free(desc);
}



void *os_contig_dma_get_virt(os_contig_dma_desc_t desc)
{
    return desc->virt_addr;
}

mw_physical_addr_t os_contig_dma_get_phys(os_contig_dma_desc_t desc)
{
    return desc->phys_addr;
}

size_t os_contig_dma_get_size(os_contig_dma_desc_t desc)
{
    return desc->size;
}


void *os_memcpy(void *dst, const void *src, unsigned int size)
{
    return memcpy(dst, src, (size_t)size);
}


/* spin lock */
os_spinlock_t os_spin_lock_alloc(void)
{
    os_spinlock_t lock;

    lock = kmem_cache_alloc(g_spin_cache, GFP_KERNEL);
    if (lock == NULL)
        return NULL;

    spin_lock_init((spinlock_t *)lock);

    return lock;
}


void os_spin_lock_free(os_spinlock_t lock)
{
    kmem_cache_free(g_spin_cache, lock);
}

void os_spin_lock(os_spinlock_t lock)
{
    spin_lock(lock);
}

void os_spin_unlock(os_spinlock_t lock)
{
    spin_unlock(lock);
}

void os_spin_lock_bh(os_spinlock_t lock)
{
    spin_lock_bh(lock);
}

void os_spin_unlock_bh(os_spinlock_t lock)
{
    spin_unlock_bh(lock);
}

int os_spin_try_lock(os_spinlock_t lock)
{
    return spin_trylock(lock);
}


static inline long
do_wait_and_clear(struct completion *x, long timeout, int state)
{
    if (!x->done) {
        DECLARE_WAITQUEUE(wait, current);

        wait.flags |= WQ_FLAG_EXCLUSIVE;
        __add_wait_queue(&x->wait, &wait);
        do {
            if (signal_pending_state(state, current)) {
                timeout = -ERESTARTSYS;
                break;
            }
            __set_current_state(state);
            spin_unlock_irq(&x->wait.lock);
            timeout = schedule_timeout(timeout);
            spin_lock_irq(&x->wait.lock);
        } while (!x->done && timeout);
        __remove_wait_queue(&x->wait, &wait);
        if (!x->done)
            return timeout;
    }
    x->done = 0;
    return timeout ?: 1;
}

static long __sched
wait_and_clear(struct completion *x, long timeout, int state)
{
    might_sleep();

    spin_lock_irq(&x->wait.lock);
    timeout = do_wait_and_clear(x, timeout, state);
    spin_unlock_irq(&x->wait.lock);
    return timeout;
}


/* @timeout: -1: wait for ever; 0: no wait; >0: wait for @timeout ms */
/* ret 0: timeout, <0 err, >0 event occur */
int32_t os_event_wait(os_event_t event, int32_t timeout)
{
    unsigned long expire;
    long ret = 0;

    if (timeout < 0)
        expire = MAX_SCHEDULE_TIMEOUT;
    else
        expire = msecs_to_jiffies(timeout);

    ret = wait_and_clear(&event->done, expire, TASK_KILLABLE);

    if (ret > INT_MAX)
        ret = 1;

    return ret;
}


os_event_t os_event_alloc(void)
{
    os_event_t event;

    event = kmem_cache_alloc(g_event_cache, GFP_KERNEL);
    if (event == NULL)
        return NULL;

    init_completion(&event->done);

    return event;
}

void os_event_free(os_event_t event)
{
    kmem_cache_free(g_event_cache, event);
}

void os_event_set(os_event_t event)
{
    complete(&event->done);
}


