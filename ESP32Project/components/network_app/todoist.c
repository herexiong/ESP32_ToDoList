#include "todoist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "ui.h"
#include "sd.h"

#define API_URL "https://api.todoist.com/rest/v2/tasks"
#define RECIVE_BUFFER_SIZE 1024*1024

char *api_token = NULL;
char *api_prj_addr = NULL;

extern const uint8_t root_cert_pem_start[] asm("_binary_root_cert_pem_start");
extern const uint8_t root_cert_pem_end[] asm("_binary_root_cert_pem_end");

todoistNode *head = NULL;

static const char *TAG = "TODOIST_TASK";

// 初始化链表头
todoistNode *init_todoist_list()
{
    todoistNode *head = (todoistNode *)malloc(sizeof(todoistNode));
    if (!head) {
        ESP_LOGE(TAG, "Failed to allocate memory for todoistNode head");
        return NULL;
    }
    head->pre = NULL;
    head->next = NULL;
    head->order = 0;
    head->content = NULL;
    head->description = NULL;
    return head;
}
//释放链表
void free_todoist_list(todoistNode *head)
{
    todoistNode *current = head;
    while (current) {
        todoistNode *next_node = current->next;

        if (current->content) free(current->content);
        if (current->description) free(current->description);
        free(current);

        current = next_node;
    }
}
//通过id寻找节点
todoistNode* find_node_by_id(todoistNode *head, const char *id)
{
    todoistNode *current = head->next; // 跳过头节点
    while (current) {
        if (strcmp(current->id , id) == 0) {
            return current; // 找到匹配的节点
        }
        current = current->next;
    }
    return NULL; // 未找到
}

//通过content来寻找节点
todoistNode* find_node_by_content(todoistNode *head, const char *content)
{
    todoistNode *current = head->next; // 跳过头节点
    while (current) {
        if (strcmp(current->content , content) == 0) {
            return current; // 找到匹配的节点
        }
        current = current->next;
    }
    return NULL; // 未找到
}

void update_node(todoistNode *node, int order, const char *content, const char *description)
{
    node->order = order;

    if (node->content) free(node->content);
    node->content = strdup(content);

    if (node->description) free(node->description);
    node->description = strdup(description);
}

void add_new_node(todoistNode *head, const char *id, int order, const char *content, const char *description)
{
    todoistNode *new_node = (todoistNode *)malloc(sizeof(todoistNode));
    if (!new_node) {
        ESP_LOGE(TAG, "Failed to allocate memory for new node");
        return;
    }

    // 初始化新节点
    new_node->id = strdup(id);
    new_node->order = order;
    new_node->content = strdup(content);
    new_node->description = strdup(description);
    new_node->pre = NULL;
    new_node->next = NULL;

    // 找到链表尾部并插入
    todoistNode *current = head;

    // 找到第一个比 new_node->order 大的节点
    while (current->next != NULL && current->next->order < order) {
        current = current->next;
    }

    // 插入新节点
    new_node->next = current->next;
    if (current->next != NULL) {
        current->next->pre = new_node;
    }
    current->next = new_node;
    new_node->pre = current;
}

void delete_node(todoistNode *node){
    if (node == NULL) {
        ESP_LOGE(TAG, "delete_node():Cannot delete a NULL node");
        return;
    }
    ESP_LOGI(TAG, "Removing outdated node with id: %s", node->id);
    //修改链表指向
    if (node->pre != NULL) {
        node->pre->next = node->next;  // 前节点的 next 指向当前节点的 next
    }
    if (node->next != NULL) {
        node->next->pre = node->pre;  // 后节点的 pre 指向当前节点的 pre
    }
    //释放内存
    if (node->id) free(node->id);
    if (node->content) free(node->content);
    if (node->description) free(node->description);
    free(node);

}

void remove_unvisited_nodes(todoistNode *head)
{
    todoistNode *current = head->next; // 跳过头节点

    while (current) {
        if (!current->is_visited) {
            ESP_LOGI(TAG, "Removing node with id: %s", current->id);
            // 从链表中删除节点
            todoistNode *to_delete = current;
            current = current->next;
            delete_node(to_delete);
        } else {
            current->is_visited = false; // 重置访问标志
            current = current->next;
        }
    }
}

//用于按order排序链表并将标记is_visited标记为false
void sortLinkedList(todoistNode *head) {
    if (!head || !(head->next) || !(head->next->next)) {
        return; // 链表为空或只有一个有效节点，无需排序
    }

    todoistNode *sorted = NULL; // 已排序链表头
    todoistNode *current = head->next; // 跳过哨兵节点

    while (current) {
        todoistNode *next = current->next; // 保存下一个节点
        current->pre = current->next = NULL; // 分离当前节点
        if (!sorted) {
            sorted = current; // 第一个节点直接放入排序链表
        } else {
            // 插入当前节点到排序链表
            todoistNode *temp = sorted;
            todoistNode *prev = NULL;

            while (temp && temp->order < current->order) {
                prev = temp;
                temp = temp->next;
            }

            if (!prev) {
                // 插入到排序链表头部
                current->next = sorted;
                sorted->pre = current;
                sorted = current;
            } else {
                // 插入到排序链表中间或尾部
                current->next = temp;
                current->pre = prev;
                prev->next = current;
                if (temp) {
                    temp->pre = current;
                }
            }
        }
        current = next; // 继续处理下一个节点
    }

    // 更新哨兵节点指向排序后的链表
    head->next = sorted;
    if (sorted) {
        sorted->pre = head;
    }
}

void parse_and_store_tasks(const char *json_data, todoistNode *head)
{
    cJSON *root = cJSON_Parse(json_data);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }

    if (!cJSON_IsArray(root)) {
        ESP_LOGE(TAG, "JSON root is not an array");
        cJSON_Delete(root);
        return;
    }

    cJSON *task = NULL;

    //标记链表节点为未访问
    todoistNode *current = head->next;
    while (current) {
        current->is_visited = false;
        current = current->next;
    }
    // sortLinkedList(head);

    cJSON_ArrayForEach(task, root)
    {
        // 提取任务的字段
        cJSON *id = cJSON_GetObjectItem(task, "id");
        cJSON *order = cJSON_GetObjectItem(task, "order");
        cJSON *content = cJSON_GetObjectItem(task, "content");
        cJSON *description = cJSON_GetObjectItem(task, "description");

        if (!id || !cJSON_IsString(id)) {
            ESP_LOGW(TAG, "Task ID is invalid, skipping...");
            continue;
        }

        // 在链表中查找是否已存在该任务
        todoistNode *existing_node = find_node_by_id(head, id->valuestring);

        if (existing_node) {
            // 更新已有节点内容
            update_node(existing_node,
                        order && cJSON_IsNumber(order) ? order->valueint : -1,
                        content && cJSON_IsString(content) ? content->valuestring : "(no content)",
                        description && cJSON_IsString(description) ? description->valuestring : "(no description)");
            existing_node->is_visited = true; // 标记为访问过
        } else {
            // 添加新节点到链表
            add_new_node(head,
                         id->valuestring,
                         order && cJSON_IsNumber(order) ? order->valueint : -1,
                         content && cJSON_IsString(content) ? content->valuestring : "(no content)",
                         description && cJSON_IsString(description) ? description->valuestring : "(no description)");
        }
    }

    // 删除未访问的节点
    remove_unvisited_nodes(head);
    sortLinkedList(head);//对节点进行排序

    cJSON_Delete(root);
}

void print_todoist_list(todoistNode *head)
{
    todoistNode *current = head->next; // 跳过头节点
    while (current) {
        ESP_LOGI(TAG, "*************************************");
        ESP_LOGI(TAG, "Task Order: %d", current->order);
        ESP_LOGI(TAG, "Content: %s", current->content);
        ESP_LOGI(TAG, "Description: %s", current->description);
        current = current->next;
    }
}

void http_post_request(const char * content)
{
    char url[100]={};
    todoistNode *node = find_node_by_content(head,content);
    if (node == NULL)
    {
        ESP_LOGE(TAG,"http_post_request can not find the node");
        return;
    }
    
    sprintf(url,"%s/%s/close",API_URL,node->id);
    // ESP_LOGI(TAG,"url:%s",url);
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        //证书
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .cert_pem = (const char *)root_cert_pem_start,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    // 设置HTTP头部
    esp_http_client_set_header(client, "Authorization", api_token);

    // 发送请求
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    // 释放客户端
    esp_http_client_cleanup(client);

}

esp_err_t http_get_request(char *buffer , const char * api_prj_addr , const char * api_token){
    	int content_length = 0;
        esp_err_t err = ESP_FAIL;
        // 配置HTTP客户端
        esp_http_client_config_t config = {
            .url = api_prj_addr,
			//证书
			.transport_type = HTTP_TRANSPORT_OVER_SSL,
			.cert_pem = (const char *)root_cert_pem_start,
            .method = HTTP_METHOD_GET,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);

		if(client != NULL){
			// 设置HTTP头部
			esp_http_client_set_header(client, "Authorization", api_token);
			err = esp_http_client_open(client, 0);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
			} else {
				content_length = esp_http_client_fetch_headers(client);
				if (content_length < 0) {
					ESP_LOGE(TAG, "HTTP client fetch headers failed");
				} else {
					int data_read = esp_http_client_read_response(client, buffer, RECIVE_BUFFER_SIZE);
					if (data_read >= 0) {
                        err = ESP_OK;
					} else {
                        err = ESP_FAIL;
						ESP_LOGE(TAG, "Failed to read response");
					}
				}
			}
			esp_http_client_close(client);
		}
		esp_http_client_cleanup(client);
        return err;
}

// 任务函数
//ToDo-> 内存线程安全问题
//ToDo-> 删除的逻辑问题，实现撤回删除功能
void todoist_task(void *pvParameters)
{
	// 在PSRAM中分配缓冲区
    char *buffer = heap_caps_malloc(RECIVE_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    head = init_todoist_list();

    todolist_syscfg_t* cfg = (todolist_syscfg_t *)pvParameters;
    
    api_token =    cfg->todoist_auth == NULL  ? NULL : (char *)malloc(sizeof(char)*(strlen("Bearer ") + strlen(cfg->todoist_auth)+1));
    api_prj_addr = cfg->todoist_prjid == NULL ? NULL : (char *)malloc(sizeof(char)*(strlen(API_URL) + strlen("?project_id=") + strlen(cfg->todoist_prjid)+1));

    if (!buffer || !head || !api_token || !api_prj_addr) {
        goto DELETE;
    }
    sprintf(api_token,"Bearer %s",cfg->todoist_auth);
    sprintf(api_prj_addr,"%s?project_id=%s",API_URL,cfg->todoist_prjid);

    while (1) {
        memset(buffer, 0, RECIVE_BUFFER_SIZE);
        if(http_get_request(buffer , api_prj_addr , api_token) == ESP_OK){
            parse_and_store_tasks(buffer, head);
            // print_todoist_list(head);
            todoist_ui_show(head->next);//跳过头节点
        }
        // 等待10s
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
DELETE:
    ESP_LOGE(TAG,"todoist_task delete");
    if(head != NULL) free_todoist_list(head);
	if(buffer != NULL) free(buffer);
	if(api_token != NULL) free(api_token);
	if(api_prj_addr != NULL) free(api_prj_addr);
    vTaskDelete(NULL);
}