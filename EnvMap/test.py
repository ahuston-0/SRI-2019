import EnvMap

test = EnvMap.EnvMap([[1, 2, 3, 4, 5], [1, 2, 3, 4, 5, 6], [5, 4, 3, 2, 1], [30, 2, 60, 1]], [1, 2])
print(test.get_buoys())
map_ = test.get_map()
for i in range(len(map_)):
    print(map_[i])