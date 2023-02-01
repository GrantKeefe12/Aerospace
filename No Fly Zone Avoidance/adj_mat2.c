#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "adj_mat2.h"

#define PI 3.141592654
#define drone_speed  20.0
#define drone_max_w  7.5
#define drone_max_p  6.0

//Coordinates of waypoints
double coordinates[27][3] = {
    {97,	 48.51667069999998,  -71.6375025},
    {98,	 48.5060947       ,  -71.63175180000002},
    {99,	 48.49211590000004,  -71.63400689999999},
    {100,	 48.51503409999999,  -71.6404442},
    {101,	 48.50053369999999,   -71.6782955},
    {102,	 48.508839500000015,  -71.6040591},
    {103,	 48.5101473,          -71.65221010000002},
    {104,    48.512991700000015,  -71.64260060000001},
    {105,	 48.5117408,          -71.6428152},
    {106,	 48.51933110000004,   -71.6229056},
    {107,	 48.49846230000001,   -71.65680879999998},
    {108,	 48.5019885,          -71.62530889999998},
    {109,	 48.520525000000006,  -71.67200080000002},
    {110,	 48.5090567,          -71.6461702},
    {111,	 48.51070570000003,   -71.6516848},
    {112,	 48.50396670000003,   -71.62981979999999},
    {113,	 48.526230800000015,  -71.6345802},
    {114,	 48.49842660000004,   -71.6425625},
    {115,	 48.5258329,          -71.6320911},
    {116,	 48.49967789999998,   -71.6758648},
    {117,	 48.49370579999999,   -71.6290012},
    {118,	 48.51035300000003,   -71.6228085},
    {119,	 48.5093153,          -71.62160689999999},
    {120,	 48.496924800000016,  -71.60340180000001},
    {121,	 48.51125569999999,   -71.6312968},
    {122,	 48.49328459999999,   -71.6664874},
    {18,     48.51156300000004,   -71.68049960000002}
};

//linked list for task information storage
typedef struct routeinfo{
    int num;
    int waypoint;
    route *next;
}route;
route *head = NULL;

typedef struct nofly_info{
    int num;
    int waypoint;
    route1 *next;
}route1;
route1 *head1 = NULL;

//structure of edge allows for information to be held
typedef struct edgeinfo{
    int from;
    int to;
}edge;

//graph structure (adjacency matrix)
typedef struct routegraph {
    int numnodes;
    edge ***edges;
}graph;

graph *generate_nofly_graph(){
    char *p,*waypoint,*pbuff, *return_way;
    int r = 0;
    int count = 0;
    FILE *fp2;

    // Get buffer to read data file into.
    pbuff = (char *) malloc(1000);
    if (pbuff == NULL) {
        printf("ERROR: Cannot get memory for buffer\n");
    }
    fp2 = fopen("no_fly_zone.txt", "r");
    if (fp2 == NULL) {
        printf("ERROR: Cannot open data file\n");
    }

    p = fgets(pbuff, 500, fp2);
    if (p == NULL) return NULL;
    while (*p != ':') p++;
    p++;

    while (*p != '\n' && *p != '\r') {
        p++;
        if (count == 1){
            *p = '\0';

            while (*p == ' ') p++;
            while (*p != ' ') p++;
            while (*p == ' ') p++;
            while (*p != ' ') p++;
            while (*p == ' ') p++;
            while (*p != ' ') p++;
            while (*p == ' ') p++;
            while (*p != ' ') p++;
            while (*p == ' ') p++;

            return_way = p;
            while (*p != ' ') p++;
            *p = '\0';
            break;
        }
        waypoint = p;
        if (waypoint[1] < 97) break;
        while (*p != ';' && count == 0){
            p++;
            if (*p == '.'){
                count = 1;
                break;
            }
        }
        *p = '\0';
        p++;  
        //store information in linked list

        //printf("waypoint %s\n", waypoint);
        
        add_nofly(r, waypoint);
        r++; 
    }

    printf("return waypoint %s\n", return_way);

    graph *g1 = create_graph(4);

    automate_nofly_edges(g1);

    print_graph(g1);

    fclose(fp2);

    return g1;
}

//reads data from txt file and creates a graph from the given information
graph *generate_route_graph(){

    char *p,*waypoint,*pbuff;
    int r = 0;
    FILE *fp1;

    // Get buffer to read data file into.
    pbuff = (char *) malloc(1000);
    if (pbuff == NULL) {
        printf("ERROR: Cannot get memory for buffer\n");
    }
    fp1 = fopen("route_info_initial.txt", "r");
    if (fp1 == NULL) {
        printf("ERROR: Cannot open data file\n");
    }

    p = fgets(pbuff, 500, fp1);
    if (p == NULL) return NULL;
    while (*p != ':') p++;
    p++;

    while (*p != '\n' && *p != '\r') {
        p++;
        waypoint = p;
        if (waypoint[1] < 97) break;
        while (*p != ';') p++;
        *p = '\0';
        p++;  
        //store information in linked list
        add_route(r,waypoint);
        r++; 
    }

    //create a blank graph
    graph *g = create_graph(r*2);

    automate_route_edges(g);

    print_graph(g);

    fclose(fp1);
    return g;
}

void add_nofly(int num, char* waypoint){

    route1 *r = (route1*)malloc(sizeof(route1));

    int w = waypoint[1];
    r-> num = num;
    r-> waypoint = w;
    r-> next = NULL;

    if (head1 == NULL){
        head1 = r;
    }else{
        route1* temp = head1;
        while (temp->next != NULL){
            temp = temp->next;
        }
        temp -> next = r;
    }
}

//linked list data storage
void add_route(int num, char* waypoint){

    route *r = (route*)malloc(sizeof(route));

    int w = waypoint[1];
    r-> num = num;
    r-> waypoint = w;
    r-> next = NULL;

    if (head == NULL){
        head = r;
    }else{
        route* temp = head;
        while (temp->next != NULL){
            temp = temp->next;
        }
        temp -> next = r;
    }
}

//prints linked list for error checking
void print_route_info(void){

    printf("all route info\n");
    route* temp;
    temp = head;

    if (temp == NULL){
        printf("list is empty\n");
    }
    while (temp != NULL){
            printf("waypoint = %d\n", temp->waypoint);
            temp = temp->next;
    }
}

//allocates the needed memory to the graph
graph *create_graph(int numnodes){

    graph *g = malloc(sizeof(*g));

    if (g==NULL){
        return NULL;
    }
    g->numnodes = numnodes;

    //allocate the matrix
    g->edges = calloc(sizeof(edge*),g->numnodes);
    if (g->edges==NULL){
        free(g);
        return NULL;
    }
    for (int i = 0; i < g->numnodes; i++){
        //calloc initializes as 0
        g->edges[i] = calloc(sizeof(edge), g-> numnodes);
        if (g->edges[i]==NULL){
            destroy_graph(g);
            return NULL;
        }
    }
    return g;
}

//frees the graph memory when finished
void destroy_graph(graph *g){

    if (g-> edges == NULL) return;
    for (int i = 0; i < g->numnodes; i++){
        if (g-> edges != NULL){
            free(g->edges[i]);
        }
    }
    free(g->edges);
    free(g);
}

//prints the vertices and edges of the graph
void print_graph(graph *g){

    printf("digraph {\n");

    for (int from = 0; from < g->numnodes; from++){
        for (int to = 0; to < g->numnodes; to++){
            if (g->edges[from][to]){
                printf("%d --(from waypoint = %d to waypoint = %d)-> %d\n", 
                from,
                g->edges[from][to]->from,
                g->edges[from][to]->to,
                to);
            }
        }
    }
    printf("}\n");
}

//allocates memory for an edge and assigns vlalues
edge *create_edge(int from,int to){
    edge *e = malloc(sizeof(*e));
    if (e == NULL){
        return NULL;
    }
    e -> from = from;
    e -> to = to;

    return e;
}

//helper for automate_edges
bool add_edge(graph *g, int from_way, int to_way, unsigned int from_node,unsigned int to_node){

    assert(g != NULL);
    assert(from_node < g-> numnodes);
    assert(to_node < g-> numnodes);

    if (has_edge(g, from_node,to_node)){
        return false;
    }

    edge *e = create_edge(from_way ,to_way);

    g->edges[from_node][to_node] = e;
  
    return true;
}
//helper for automate_edges
bool has_edge(graph *g, unsigned int from_node,unsigned int to_node){

    assert(g != NULL);
    assert(from_node < g-> numnodes);
    assert(to_node < g-> numnodes);

    return g->edges[from_node][to_node];
}

void automate_nofly_edges(graph *g){
    route1* temp;
    temp = head1;

    while (temp->next != NULL){
        add_edge(g, temp->waypoint, temp->next->waypoint, temp->num, temp->next->num);
        temp = temp->next;
    }   
}

void automate_route_edges(graph *g){
    route* temp;
    temp = head;

    while (temp->next != NULL){
        add_edge(g, temp->waypoint, temp->next->waypoint, temp->num, temp->next->num);
        temp = temp->next;
    }   
}

