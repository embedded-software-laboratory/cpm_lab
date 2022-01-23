#import networkx as nx
import igraph
from igraph import *
import matplotlib.pyplot as plt

#G = nx.DiGraph()


#G.add_edges_from([(20, 9) ,(9, 8) ,(8, 7) ,(7, 13) ,(7, 6) ,(13, 12) ,(12, 15) ,(12, 14) ,(15, 11) ,(11, 10) ,(10, 8) ,(6, 0) ,(0, 1) ,(1, 2) ,(2, 3) ,(3, 4) ,(4, 5) ,(4, 17) ,(5, 16) ,(16, 11) ,(14, 23) ,(23, 21) ,(21, 22) ,(22, 18) ,(18, 19) ,(19, 20) ,(17, 21)])
#labels = {20:1,9:2,8:3,7:4,13:5,12:6,15:7,11:8,10:9,6:5,0:6,1:7,2:8,3:9,4:10,5:11,16:12,14:7,23:8,21:9,22:10,18:11,19:12,17:11}

#pos = nx.spring_layout(G, seed=2)
#nx.draw(G, pos)
#nx.draw_networkx_labels(G, pos, labels)

#nx.draw_networkx_labels(G, nx.spring_layout(G), labels)


g = Graph([(20, 9) ,(9, 8) ,(8, 7) ,(7, 13) ,(7, 6) ,(13, 12) ,(12, 15) ,(12, 14) ,(15, 11) ,(11, 10) ,(10, 8) ,(6, 0) ,(0, 1) ,(1, 2) ,(2, 3) ,(3, 4) ,(4, 5) ,(4, 17) ,(5, 16) ,(16, 11) ,(14, 23) ,(23, 21) ,(21, 22) ,(22, 18) ,(18, 19) ,(19, 20) ,(17, 21)], directed=True)
tmplabel = {20:1,9:2,8:3,7:4,13:5,12:6,15:7,11:8,10:9,6:5,0:6,1:7,2:8,3:9,4:10,5:11,16:12,14:7,23:8,21:9,22:10,18:11,19:12,17:11}
labels = []
for i in range(1,len(tmplabel)):
    print(i)
    labels += [tmplabel[i]]
g.vs["label"] = labels
print(g.vs["label"])
layout = g.layout("kk")
fig, ax = plt.subplots()
plot(g, layout=layout)
#plt.show()