import Buoy_List

test = Buoy_List.BuoyList()
for i in range(7):
    test.add_buoy([1,1,"red"])
test = test.get_confirmed_buoys()
for item in test:
    print(item)