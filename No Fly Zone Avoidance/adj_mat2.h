//Graph implimentaion

typedef struct routeinfo route;

typedef struct nofly_info route1;

typedef struct routegraph graph;

typedef struct edgeinfo edge;

graph *generate_nofly_graph();

graph *generate_route_graph();

void add_nofly(int num, char* waypoint);

void add_route(int num, char* waypoint);

void print_route_info(void);

graph *create_graph(int numnodes);

void destroy_graph(graph *g);

void print_graph(graph *g);

edge *create_edge(int from,int to);

bool add_edge(graph *g, int from_way, int to_way, unsigned int from_node,unsigned int to_node);

bool has_edge(graph *g, unsigned int from_node,unsigned int to_node);

void automate_nofly_edges(graph *g);

void automate_route_edges(graph *g);