#include <stdio.h>
#include <stdlib.h>
#include "queue.h"

int empty(struct queue_t * q) {
        if (q == NULL) return 1;
	return (q->size == 0);
}

void enqueue(struct queue_t * q, struct pcb_t * proc) {
        /* TODO: put a new process to queue [q] */
         if (q->size >= MAX_QUEUE_SIZE) {
        fprintf(stderr, "[enqueue] Queue full. Cannot enqueue process PID: %d\n", proc->pid);
        return;
    }

    // Check if proc is already in queue
    for (int i = 0; i < q->size; i++) {
        if (q->proc[i]->pid == proc->pid) {
            return; 
        }
    }

    q->proc[q->size++] = proc;
}

struct pcb_t * dequeue(struct queue_t * q) {
        /* TODO: return a pcb whose prioprity is the highest
         * in the queue [q] and remember to remove it from q
         * */
	if (empty(q)) return NULL;

    int highest_idx = 0;
    for (int i = 1; i < q->size; i++) {
        if (q->proc[i]->prio < q->proc[highest_idx]->prio) {
            highest_idx = i;
        }
    }

    struct pcb_t * selected_proc = q->proc[highest_idx];

    // Shift elements left to fill the gap
    for (int i = highest_idx; i < q->size - 1; i++) {
        q->proc[i] = q->proc[i + 1];
    }
    q->size--;

    return selected_proc;
}


