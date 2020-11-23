
% function to visualize the centerlines of the lanelet maps
function drawCenterline(r_tree,map)

hold on
idList = r_tree.get_idList();
Iterator = idList;
while Iterator.hasSucc()
    Iterator = Iterator.get_Succ();
    keyId = Iterator.get_Content();
    lanelet = map.Search(keyId).value;
    lanelet.drawCenterline();
end