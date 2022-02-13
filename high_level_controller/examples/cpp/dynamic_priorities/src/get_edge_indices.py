# full lane graph edges:
full_graph_start = [0,1,2,3,4,5,11,10,9,20,8,7,12,11,13,13,6,5,17,18,19,15,16,14]
full_graph_end = [1,2,3,4,5,6,0,11,10,9,9,8,13,12,14,7,7,15,18,19,20,16,17,15]

# example edge subset
oval_vert_beg = [0,1,2,3,4,5,6,7,8,9, 10,11]
oval_vert_end = [1,2,3,4,5,6,7,8,9,10,11, 0]

oval_hor_beg = [11,12,13,14,15,16,17,18,19,20,9, 10]
oval_hor_end = [12,13,14,15,16,17,18,19,20, 9,10,11]

indices_vert = []
indices_hor = []
# get indices of example subset edges
index = 0
for edge in full_graph_start:
    print(index, " : ", edge, " -> ", full_graph_end[index])
    index_subset_1 = 0
    index_subset_2 = 0
    for edge_subset in oval_vert_beg:
        if edge == edge_subset and full_graph_end[index] == oval_vert_end[index_subset_1]:
            indices_vert.append(index)
        index_subset_1 +=1
    for edge_subseth in oval_hor_beg:
        if edge == edge_subseth and full_graph_end[index] == oval_hor_end[index_subset_2]:
            indices_hor.append(index)
        index_subset_2 +=1
    index+=1


print(indices_vert)
print(indices_hor)
print(len(indices_vert))
