/*
 * Copyright 2017 Brian Starkey <stark3y@gmail.com>
 *
 * Queue implementation, inspired by http://www.1024cores.net/home/lock-free-algorithms/queues/intrusive-mpsc-node-based-queue
 *
 * I believe that this is thread-safe with multiple producers and a single
 * consumer, but I'm not very clever, so maybe don't take my word for it.
 */
#ifndef __QUEUE_H__
#define __QUEUE_H__

struct queue {
	struct queue_node *next;
	struct queue_node *last;
};

struct queue_node;

void dump_queue(struct queue *queue);
void queue_enqueue(struct queue *queue, struct queue_node *node);
struct queue_node *queue_dequeue(struct queue *queue);

#endif /* __QUEUE_H__ */
