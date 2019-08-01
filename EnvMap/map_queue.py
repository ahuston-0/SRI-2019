class MapQueue:
    def __init__(self):
        self.list = []

    def is_empty(self):
        return self.list == []

    def push(self, item):
        self.list.insert(0, item)

    def pop(self):
        return self.list.pop()


test = MapQueue()
for i in range(100):
    test.push(i)

while not test.is_empty():
    print(test.pop())