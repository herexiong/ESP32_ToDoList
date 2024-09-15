#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/stat.h>

#define uint16_t wint_t

#define LIST_MAX_NUM 1000
#define CONTENT_MAX_LEN 1000
const char *path = ".//ToDoList.txt";

typedef struct{
	unsigned char *content;//用于存储内容，内存动态分配
	struct toDoListNode *next;
} toDoListNode_t;

toDoListNode_t head ;

//1代表第一个数据节点
toDoListNode_t* new_ToDoListNode(uint16_t pos , size_t len){//从1开始
	uint16_t i = 1;
	toDoListNode_t *cur = &head;
	while (pos > i)
	{
		i++;
		cur = cur->next;
	}
	toDoListNode_t *temp = cur->next == NULL ? NULL : cur->next;
	cur->next = (toDoListNode_t *)malloc(sizeof(toDoListNode_t));
	cur = cur->next;
	cur->content = (unsigned char *)malloc(sizeof(unsigned char)*(len));
	cur->next = temp;
	head.content++;
	return cur;
}

bool ToDoList_init(const char *toDoListPath){
	head.content = (unsigned char *)malloc(sizeof(unsigned char)*4);
	head.content = 0;
	head.next = NULL;

	FILE *file = fopen(toDoListPath, "r");
    if (!file) {
        perror("file is not existed,ready to creat a new one");
        file = fopen(toDoListPath, "wb");
		if (!file) return false;
		else{
			fclose(file);
			return true;
		}
    }

	unsigned char buffer[CONTENT_MAX_LEN];
	size_t length;
	while (fgets(buffer,  sizeof(buffer), file) != NULL) {
		length = strlen(buffer);
		// 去掉换行符
		if (length > 0 && buffer[length - 1] == '\n') {
			if (length >1 && buffer[length-2] == '\r')//window换行符是\r\n
			{
				buffer[length - 2] = '\0';
				length--;
			}
			buffer[length - 1] = '\0';
		}else{
			length++;
			buffer[length - 1] = '\0';
		}

		toDoListNode_t* node = new_ToDoListNode((uint16_t)head.content+1,length);
		strcpy(node->content,buffer);
	}

	fclose(file);
	return true;
}

bool ToDoList_update_file(void){
	FILE *file = fopen(path, "w");
	if (file == NULL)
	{
		return false;
	}
	toDoListNode_t *cur = head.next;
	while (cur != NULL)
	{
		fprintf(file, "%s\n",cur->content);
		cur = cur != NULL ? cur->next : NULL;
	}
	fclose(file);
	return true;
}

bool ToDoList_insert(int pos ,const unsigned char *content , int content_len){
	if (pos > head.content || content_len > CONTENT_MAX_LEN)
	{
		return false;
	}
	toDoListNode_t *node = new_ToDoListNode(pos+1,content_len);
	strcpy(node->content,content);

	ToDoList_update_file();
	return true;
}

// 删除链表中的一个节点
bool ToDolist_delete(int pos) {
    if (pos < 1 || pos > head.content) {
        return false;
    }

    toDoListNode_t *current = &head;
    toDoListNode_t *previous = NULL;

    // 跳过链表头节点
    current = current->next;
    pos--;

    // 遍历链表，找到要删除的节点
    while (current != NULL && pos > 0) {
        previous = current;
        current = current->next;
        pos--;
    }

    // 如果找到了要删除的节点
    if (current != NULL) {
        if (previous != NULL) {
            previous->next = current->next;
        } else {
            head.next = current->next;
        }
        free(current->content);
        free(current);
        // 更新链表长度
        head.content--;
        ToDoList_update_file();
        return true;
    }

    return false;
}



int main(void){
	ToDoList_init(path);
	const unsigned char *str= "ToDoList";
	ToDoList_insert(head.content,str,strlen(str)+1);
	// ToDolist_delete(head.content);
	toDoListNode_t *node = head.next;
	while (node != NULL)
	{
		printf("%s\n",node->content);
		node = node->next;
	}
	printf("%d\n",head.content);
	
	return 0;
}