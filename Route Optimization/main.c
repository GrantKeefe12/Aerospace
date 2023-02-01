#include "adj_mat.c"

int main(){

    graph *g = generate_graph();

    waypoint *w = search_graph(g);

    printf("BEST FLIGHT PLAN: profit = %f time = %f\n", w -> value, w -> time);

    while (w != NULL){
        printf("From %d to %d\n", w->from, w->to);
        w = w->previous;
    }
}