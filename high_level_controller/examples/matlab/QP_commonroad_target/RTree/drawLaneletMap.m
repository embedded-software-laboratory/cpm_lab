% function to visualize left and right bound of each lanelet in the map
function fig =drawLaneletMap(r_tree,map)
fig = figure;
hold on
idList = r_tree.get_idList;
Iterator = idList;
while Iterator.hasSucc()
    Iterator = Iterator.get_Succ();
    keyId = Iterator.get_Content();
    lanelet = map.Search(keyId).value;
    lanelet.drawLanelet();
    1;
end


hold off
end


