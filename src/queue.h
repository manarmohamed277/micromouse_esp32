#ifndef QLINKED_H
#define QLINKED_H

typedef struct {
    int x;
    int y;
} queueEntry;

typedef struct queueNode {
    queueEntry entry;
    struct queueNode *next;
} queueNode;

typedef struct {
    queueNode *front;
    queueNode *rear;
    int size;
} queue;

#ifdef __cplusplus
extern "C" {
#endif

void creatQueue(queue *qp);
void append(queueEntry item, queue *qp);
void serve(queueEntry *item, queue *qp);
int queueEmpty(queue *qp);
int queueFull(queue *qp);
void clearQueue(queue *qp);
void traverseQueue(queue *qp, void (*ptr)(queueEntry));

#ifdef __cplusplus
}
#endif

#endif // QLINKED_H
