import Buoy_List

test = Buoy_List.BuoyList()
for i in range(8):
    test.add_buoy([1,1,"red"])
print(test.get_buoys())