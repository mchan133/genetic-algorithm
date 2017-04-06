import heapq
# code used from https://docs.python.org/2/library/heapq.html

class My_PQ:

    def __init__(self):
        self.pq = []
        self.entries = {}
        self.REMOVED = (-1,-1)

    def put2(self, priority, entry):
        if entry in self.entries:
            self.remove(entry)
        item = [priority, entry]
        self.entries[entry] = item
        heapq.heappush(self.pq, item)

    def put(self, e):
        priority = e[0]
        entry = e[1]
        if entry in self.entries:
            self.remove(entry)
        item = [priority, entry]
        self.entries[entry] = item
        heapq.heappush(self.pq, item)

    def empty(self):
        return len(self.pq) == 0

    def qsize(self):
        return len(self.pq)

    def contains(self, entry):
        return entry in self.entries


    def remove(self, entry):
        item = self.entries.get(entry, None)
        if not item:
            return
        item[-1] = self.REMOVED

    def get(self):
        while self.pq:
            item = heapq.heappop(self.pq)
            if item[1] is not self.REMOVED:
                del self.entries[item[1]]
                return item
        raise KeyError("pop: priority queue empty!")

    # should work properly
    def peek(self):
        while self.pq:
            item = heapq.heappop(self.pq)
            if item[1] is not self.REMOVED:
                heapq.heappush(self.pq, item)
                return item
        #raise KeyError("peek: priority queue empty!")
        #return (float('inf'), (-1,-1))



if __name__ == "__main__":
    pq = My_PQ()

    pq.put(1,(1,1))
    print(pq.contains((1,1)))
    pq.put(2,(2,2))
    pq.put(0,(26,26))
    print(pq.pq)
    pq.put(26,(26,26))
    pq.put(0,(999,999))
    print(pq.pq)
    print(pq.entries)
    print(pq.peek())
    print(pq.pq)
    pq.put(27,(999,999))
    print(pq.pq)
    print(pq.get())
    print(pq.pq)
    print(pq.entries)
    print(pq.contains((1,1)))
