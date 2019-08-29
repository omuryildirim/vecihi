from collections import defaultdict


def find_euler_tour(graph):
    """
    Calculate Euler tour.
    Reference: http://web.eecs.umich.edu/~pettie/matching/Edmonds-Johnson-chinese-postman.pdf

    :param graph: Reeb Graph of given map.
    """
    def find_tour(u, E, tour):
        if u == 19:
            print(u)
        for (a, b) in E:
            if a == u:
                E.remove([a, b])
                find_tour(b, E, tour)
            elif b == u:
                E.remove([a, b])
                find_tour(a, E, tour)
        tour.insert(0, u)

    tour = []
    find_tour(graph[0][0], graph, tour)
    return tour
