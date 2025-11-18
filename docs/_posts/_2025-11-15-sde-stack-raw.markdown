


# Stack

## 捣腾

定义：2个stack，所有元素依次由1放到2

颠倒顺序

O(2n)

## 1. How to sort numbers with 2 stacks

(待补全)

witout duplicated values:

    global min
    stack 1 (input)
    stack 2 (result | buffer)
with duplicated values:
    
    global min
    count of global min
    stack 1
    stack 2

## 2. How to implement a queue by using two stacks

```java
class MyQueue{
    stack1(push)=
    stack2(pop)=

    offer(x){
        stack2.push(x)
    }
    poll(){
        if(!stack2.isEmpty()){
            return stack2.pop();
        }else{
            if(!stack1.isEmpty()){
                return null;
            }
        }
        捣腾;
        return stack2.pop();
    }
}
```
Amortized time complexity: 只有在一次操作能换取后续操作时间复杂度降低才用这个

均摊时间复杂度：O(3n)/(O(1)*n)=O(3)=O(1) # **不明白**

## 3. How to implement the min() function when using stack with time complexity O(1) for each operation?

在普通stack push pop top ， 加上min()(返回最小值)，并要求O(1)实现

用两个stack：

    同时压同时弹
    stack1压入元素
    stack2记录每个元素压入stack1同时的最小值
    当stack1弹出时，stack2同位置为当前最小
    空间O(n)
    时间O(1)

优化：

    只在更小值出现时才压入Stack2
    Stack1压入的等于当前最小值时也要压入stack2 否则无法判断当前stack2中出现的最小值的stack1来源

    pop时：
    stack1pop的 >  min stack2不动
    stack1pop的 ==  min stack2pop

    只有worstcase 空间O(n)
优化
    
    三个stack 
    前两个功能相同 但stack2不再压入相同的min
    第三个记录min压入stack2时stack1的size 
    stack2.size == stack3.size 永远true

    pop时：
    stack1pop的 >  min stack2不动
    stack1pop的 ==  min stack2 && stack1.size == stack3对应位置的值  pop stack2

## 4. 用多个stack实现双端队列

懒得记了

### use 3 stacks to improve the time complexity of remove(), always amortized O(1)

stack1 装前半段 stack2装结果 stack3装后半段

每次操作stack2捣腾到1和3

**不明白**： 什么叫**换来多少次O(1)**?


## 总结

stack特性：倒腾一次反序，两次回去

很多题思路都是在这个过程中做操作（第二次倒腾回去的是modified）

stack用来做什么？

    从中缀表达式a + b 到后缀表达式a b +
    reversed polish notation
    遇到符号弹两个数字再压回stack




