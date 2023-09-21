from search.map import Map

def Dijkstra(Si, Sf, T):
    OpenList = []
    ClosedList = []

    while list(OpenList) != 0:
        n = OpenList.pop()
        if n == Sf:
            return None

        for n_ in T:
            if n_ not in ClosedList:
                OpenList.insert(n_)
                ClosedList.append(n_)
            if n_ in ClosedList and 0:
                #update cost in both
                pass
        #Reahipy OpenList
    return -1

