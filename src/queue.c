//
// Created by User on 8/18/2025.
//

#include "queue.h"
#include<stdio.h>
#include <malloc.h>
void creatQueue(queue*qp){
    qp->front=  NULL;
    qp->rear=NULL;
    qp->size=0;
}

void append(queueEntry item,queue*qp){
    queueNode *pn=(queueNode*)malloc(sizeof (queueNode));
    pn->entry=item;
    pn->next= NULL;

    if(qp->rear==NULL)
        qp->front=pn;
    else
        qp->rear->next=pn; //run time error in the first time

    qp->rear=pn;
    qp->size++;

}

void serve(queueEntry*item,queue*qp){
    *item=qp->front->entry;
    queueNode *ptr=qp->front;
    qp->front=qp->front->next;
    free(ptr);
    if(qp->front==NULL) //if there is only one element
        qp->rear=NULL;
    qp->size--;
}

int queueEmpty(queue*qp){
    return (qp->front==NULL);
}
int queueFull(queue*qp){
    return 0;
}
int queueSize(queue *qp){
    return qp->size;
}

void clearQueue(queue*qp){
    queueNode *ptr=qp->front;
    while(ptr){
        ptr=ptr->next;
        free(qp->front);
        qp->front=ptr;
    }
    qp->rear=NULL;
    qp->size=0;
}

void clearQueueMODYFIED(queue*qp){
    while(qp->rear){
        qp->rear=qp->front->next;
        free(qp->front);
        qp->front=qp->rear;
    }
    qp->size=0;
}


void traverseQueue(queue*qp,void(*ptr)(queueEntry)){
    queueNode *qptr=qp->front;
    while(qptr){
        (*ptr)(qptr->entry);
        qptr=qptr->next;
    }
}


