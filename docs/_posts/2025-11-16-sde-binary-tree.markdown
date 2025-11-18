---
title: Binary Tree 笔记与重点整理
layout: post
date: 2025-11-15
categories: [Java & SDE]
tags: [Binary Tree, BST, Recursion]
---

# Binary Tree

## 一、遍历（Traversal）

二叉树的遍历分为三种基本方式：

| 遍历方式 | 访问顺序 | 常用场景 |
|-----------|------------|-----------|
| Preorder | 根 → 左 → 右 | 拷贝、构建树结构 |
| Inorder | 左 → 根 → 右 | 二叉搜索树 (BST) 输出有序结果 |
| Postorder | 左 → 右 → 根 | 删除节点、释放内存、后序处理 |

**示例：**
```java
void preorder(TreeNode root) {
    if (root == null) return;
    System.out.print(root.key + " ");
    preorder(root.left);
    preorder(root.right);
}

void inorder(TreeNode root) {
    if (root == null) return;
    inorder(root.left);
    System.out.print(root.key + " ");
    inorder(root.right);
}

void postorder(TreeNode root) {
    if (root == null) return;
    postorder(root.left);
    postorder(root.right);
    System.out.print(root.key + " ");
}
```

---

## 二、Balanced Binary Tree（平衡二叉树）

**定义：**  
对于树中的任意一个节点，其左右子树的高度差不超过 1。

**性质：**

-   空树是平衡的；
    
-   平衡不代表完全；
    
-   典型例子：AVL Tree、红黑树。
    

**判断平衡：**  
递归计算左右子树高度，若任意节点左右高度差 > 1，则不平衡。

---

## 三、Complete Binary Tree（完全二叉树）

**定义：**

-   除最后一层外，每一层都必须被填满；
    
-   最后一层的节点必须靠左连续排列；
    
-   只有最后一层的右侧可以出现空节点。
    

**常见用途：**

-   实现堆（Heap）；
    
-   优化存储结构（数组形式表示二叉树）。
    

---

## 四、Binary Search Tree（BST，二叉搜索树）

**定义：**

-   每个节点的值大于左子树所有节点；
    
-   每个节点的值小于右子树所有节点；
    
-   不允许重复值；
    
-   其 **中序遍历 (inorder)** 的结果是 **递增序列**。
    

**示例：**

```markdown
        5
      /   \
     3     8
    / \     \
   1   4     11
```

Inorder 遍历结果：

```
1, 3, 4, 5, 8, 11
```

**BST 的性质：**

| 操作 | 平均复杂度 | 最坏情况 |
| --- | --- | --- |
| Search | O(log n) | O(n) |
| Insert | O(log n) | O(n) |
| Delete | O(log n) | O(n) |

---

## 五、常见面试题

### Question 0: Get Height of the Binary Tree

**思路：**  
递归计算每个节点左右子树的高度，取最大值 +1。

**递归定义：**

```java
int getHeight(TreeNode root) {
    if (root == null) return 0;                  // base case
    int left = getHeight(root.left);             // left subtree height
    int right = getHeight(root.right);           // right subtree height
    return Math.max(left, right) + 1;            // current height
}
```

**时间复杂度：** O(n)  
**空间复杂度：** O(height)

---

### Question 1: Check if Binary Tree is Balanced

**思路：**

1.  对每个节点递归求左右子树高度；
    
2.  若任意节点的高度差 > 1，则不平衡；
    
3.  若所有节点平衡，则整个树平衡。
    

**递归公式：**

```pgsql
height(root) = max(height(left), height(right)) + 1
balanced(root) = |height(left) - height(right)| ≤ 1
```

**实现：**

```java
boolean isBalanced(TreeNode root) {
    if (root == null) return true;
    return getHeight(root) != -1;
}

int getHeight(TreeNode root) {
    if (root == null) return 0;
    int left = getHeight(root.left);
    if (left == -1) return -1;
    int right = getHeight(root.right);
    if (right == -1) return -1;
    if (Math.abs(left - right) > 1) return -1;
    return Math.max(left, right) + 1;
}
```

**优化：**  
通过返回 `-1` 早停，避免重复计算子树高度。

---

### 经典题：Print BST keys in the given range

**题意：**  
给定一个 BST 和区间 `[min, max]`，打印所有位于此范围内的节点。

**思路：**  
利用 BST 的有序性：

-   若当前节点值 < min，则只需遍历右子树；
    
-   若当前节点值 > max，则只需遍历左子树；
    
-   否则打印当前节点并遍历两边。
    

**实现：**

```java
void printRange(TreeNode root, int min, int max) {
    if (root == null) return;
    if (root.key > min) {
        printRange(root.left, min, max);
    }
    if (root.key >= min && root.key <= max) {
        System.out.print(root.key + " ");
    }
    if (root.key < max) {
        printRange(root.right, min, max);
    }
}
```

**时间复杂度：** O(k + log n)，其中 k 为输出的节点数。

---

## 六、总结

| 概念 | 定义 | 特性 |
| --- | --- | --- |
| Binary Tree | 每个节点最多两个子节点 | 不要求排序 |
| Balanced Tree | 任意节点左右高度差 ≤ 1 | AVL、Red-Black Tree |
| Complete Tree | 仅最后一层右侧可空 | 常用于堆 |
| Binary Search Tree | 左 < 根 < 右 | 中序遍历有序 |
| 高度计算 | 递归取 max(left, right)+1 | O(n) |
| 平衡检测 | 同时计算高度与平衡性 | O(n) |
| 区间打印 | 利用 BST 有序性剪枝 | O(k + log n) |

---
