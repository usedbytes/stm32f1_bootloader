/*
 * Copyright 2017 Brian Starkey <stark3y@gmail.com>
 *
 * Queue implementation, inspired by http://www.1024cores.net/home/lock-free-algorithms/queues/intrusive-mpsc-node-based-queue
 */

#include <libopencm3/cm3/sync.h>

#include <stdint.h>
#include <stdio.h>

#include "queue.h"

struct queue_node {
	struct queue_node *next;
};

static uint32_t atomic_exchange(uint32_t *ptr, uint32_t value)
{
	uint32_t ret;
	do {
		ret = __ldrex(ptr);
	} while (__strex(value, ptr));

	return ret;
}

static bool compare_and_swap(uint32_t *ptr, uint32_t compare, uint32_t swap)
{
	while(true) {
		uint32_t val = __ldrex(ptr);
		if (val != compare) {
			return false;
		}

		if (!__strex(swap, ptr)) {
			return true;
		}
	}
}

void dump_queue(struct queue *queue)
{
	struct queue_node *node = queue->next;

	printf("List %p\r\n", queue);
	while (node) {
		printf(" %p\r\n", node);
		if (node->next == node) {
			printf("Loop detected: %p\r\n", node);
			break;
		}
		node = node->next;
	}
	printf(" `-> last %p\r\n", queue->last);
}

void queue_enqueue(struct queue *queue, struct queue_node *node)
{
	struct queue_node *prev;

	node->next = NULL;

	prev = (struct queue_node *)atomic_exchange((uint32_t *)(&queue->last), (uint32_t)(node));
	if (prev)
		prev->next = node;
}

struct queue_node *queue_dequeue(struct queue *queue)
{
	struct queue_node *last, *node = queue->next;

	if (!node) {
		return NULL;
	}

	last = queue->last;
	if (last == node) {
		compare_and_swap((uint32_t *)(&queue->last), (uint32_t)node, (uint32_t)(queue));
	}
	compare_and_swap((uint32_t *)(&queue->next), (uint32_t)node, (uint32_t)(node->next));

	node->next = NULL;
	return node;
}

#if 0
void test(void)
{
	int i;
	struct queue_node *node;

	for (i = 0; i < 3; i++) {
		queue_queue(&myqueue, &nodes[i]);
	}

	dump_queue(&myqueue);

	node = queue_dequeue(&myqueue);
	printf("dq: %p\r\n", node);

	dump_queue(&myqueue);

	node = queue_dequeue(&myqueue);
	printf("dq: %p\r\n", node);

	dump_queue(&myqueue);

	queue_queue(&myqueue, &nodes[i]);

	dump_queue(&myqueue);

	node = queue_dequeue(&myqueue);
	printf("dq: %p\r\n", node);

	node = queue_dequeue(&myqueue);
	printf("dq: %p\r\n", node);

	node = queue_dequeue(&myqueue);
	printf("dq: %p\r\n", node);

	dump_queue(&myqueue);

	for (i = 5; i < 5 + 3; i++) {
		queue_queue(&myqueue, &nodes[i]);
	}

	dump_queue(&myqueue);
}
#endif
