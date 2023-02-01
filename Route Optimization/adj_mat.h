//Graph implimentaion

typedef struct routeinfo route;

typedef struct mygraph graph;

graph *generate_graph();

float find_value(float distance, float weight, float people, float profit);

float find_time(float distance, float weight, float people);

float find_distance(int s, int f);

float havershine(double startlong, double startlat, double finishlong, double finishlat);

void add_route(int numroute, int numpeople, float w, float pr, char* start, char* finish);

void print_route_info(void);

graph *create_graph(int numnodes);

void destroy_graph(graph *g);

void print_graph(graph *g);

void automate_edges(graph *g);

bool add_edge(graph *g, float value, float time, unsigned int from_node,unsigned int to_node);

bool has_edge(graph *g, unsigned int from_node,unsigned int to_node);

//BFT implementation

typedef struct node waypoint;

waypoint *create_node(int from, int to, float time, float value);

void add_node(waypoint *node);

waypoint *pop_node(void);


