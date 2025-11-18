# Graph

## from Binary tree to graph


### General Tree (n-ary tree)

也用节点表示
二叉树也是一种narytree
    
```java
List<GraphNode> neighbors;
```

区别是graph没有可以用来表示整个图的root

所以用`List<GraphNode>`表示图

## Graph

图是顶点和边的集合

    G=E+V
    Graph = Edges + Vertices

方向

    边可以是单向/无向=双向
    有向图/无向图
    
环

    有环/无环
    只在有向图讨论环

联通性

    联通图：每个顶点都连在一起，没有独立部分

应用：
    
    推荐算法
    人是节点关系是边
    搜索关系

## 如何表示图

### 邻接矩阵
```java
adj[i][j] == 1
表示从i到j有一条边
```

对于无向图，邻接矩阵对角线对称

对于V个顶点的无向图边的数量

    min=0
    max = v(v-1)/2 ~ O(v^2)

Dense/sparse Graph

邻接矩阵对于稀疏图效率低

### 邻接表

list of list

graph.get(i)

## 图的遍历

    
----

## 树的遍历

    根据遍历方式：inorder,preorder,postorder
    constructTree
    deconstructTree

### BST Binary Search Tree

**默认没有有重复值**

    inorder 是sortedarray
    可以search(), insert(), remove()
    都是O(height) [O(n) ~ O(logn)]


search()

在每个node对比当前nodealue和target，左小右大

### self-balancing Binary Search Tree

为了获得O(logn), 使用self-balancing BST, 例如AVL Tree，红黑树

    Red-black Tree
    Java classes: TreeMap/TreeSet

### BST with dupliacated node

```Java
class TreeNode{
    int key;
    int cnt;
    ...
}
```

### Search 的实现

直接return找到的node，没找到null

代码：
```Java
```

实现为tail recursion

递归到最后才返回

recursive call is the last execution statement

如果`return isBST(left) && isBST(right);`,不是tail recursion

已经证明tail recursion 可以转换为iteration，可以节省callstack上的空间复杂度

但是没有固定转化公式


### insert

返回TreeNode（新的tree头）

先serach到正确的位置

insert到一个空位，而不是取代/修改现有的

难点：找到位置后回溯到上一循环找父节点

    1. 在一个节点curr往下看而不进入 用循环实现
    2. curr走到null回溯，每次维护previous 用循环实现

也可以用递归实现：

```Java
```
写出代码并自己填写callstack

面试题应该用iterative解，因为