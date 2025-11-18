# Queue 
**FIFO**
## queue
create:
```Java
Queue<Integer> queue = new ArrayQueue<>();
```
```
insert  add     /   offer

remove  remove  /   poll

examine element /   peek
```

对于空对象第一组throw exception

第二组不会throw exception，而是return null

99%用后面这组，除非特别需要产生exception

## deque 双端队列
create:
```Java
Deque<Integer> deque = new ArrayDeque<>();
```

queue和stack都用这个  具体FIFO/LIFO看你定义，都能实现

offerFirst() offerLast()     插入？

peekFirst() peekLast()

pollfirst() ...

... ...

## Deque vs. Queue

Deque 能实现所有Queue所有功能

Deque 也有 peek() poll() ... 但是还是要用peekFirst() 为了清晰

不要写：
```Java
Deque<Integer> queue = new ArrayQueue<>();
```

## Stack
```Java
Deque<Integer> stack = new ArrayStack<>();
```

---
所有API都是O(1)

Deque不能有null

---
Integer 是对象  int是primitive type

overhead和locality

---

## Practice

### Stack using Linked List

单链表就行

head作为栈底

如果tail为栈底，访问倒数第二个需要O(n),head作为栈底则为O(1)

定义API
```Java
public class Stack{
    public Integer pop();   # 如用int不能返回null
    public peek();
    public boolean push(); #能push ->True 不能
    public int size();
    public boolean isEmpty();
}
```
补全：
```Java
public class Stack{
    int size;
    ListNode head;

    public Stack{
        this.head = null;
        this.size = 0;
    }

    public Integer pop(){
        if (head == null) return null;
        ListNode result = head;
        head = head.next;
        result.next = null;
        size--;
        return result.value;

    }
    public peek(){
        if (head == null) return null;
        return head.value;
    }
    public boolean push(int ele){
        ListNode newNode = new ListNode(ele);
        newNode.next = head;
        size++；
        head = newNode;

        return true;
    }
    public int size(){
        return size;
    }
    public boolean isEmpty(){
        return head == null; # or size() == 0;
    }
}
```

### Stack using Linked List

head初始化为-1

pop() head--;

push() head++;

```Java
public class Stack{
    int[] array;
    int head;
    public Stack(int capa){
        array = new int[capa];
        head = -1;
    }
    public Integer pop(){
        if (head = -1) return null; # OR isempty() == true
        result = array[head];
        head--;
        return result;
        # OR 三目运算符 return isEmpty() ? null : array[head--];
    }
    public peek(){
        return isEmpty() ? null : array[head];
    }
    public boolean push(int e){
        if (head = -1) return null;
        array[++head] == ele;
        return true;
        # 怎么用三目运算符写？
        
    } 
    public int size(){
        return head+1;
    }
    public boolean isEmpty(){
        return size()==0;
    }
    public boolean isFull(){
        return size()==array.length();
    }
}
```

### Queue using Linked List
只用单链表 定义head和tail

```java
public class Queue{
    int size;
    ListNode head;
    ListNode tail;

    # 省略构造函数
    # 检查 size=0、1、2
    public Integer pop(){
        if (head == null) return null;
        if (head.next = null){
            head = tail = null;
            size--;
        }
        reult  = head.value;
        head = head.next;
    }
    peek() return head.value;
    push(int ele) {
        if head==null head = new ListNode; tail=head;
        else tail.next = new ListNode(ele); tail=tail.next;
        size++;
    }
    size()
    isempty()
    isfull()

}

```

### Queue using Linked Array
环形数组
head = head + 1 == array.length ? 0 :head+1; # 没明白
tail = tail + 1 == array.length ? 0 ： tail + 1;

head指向头 pop [head] (例外head==null)
tail指向尾+1 push [tail] 

head == tail 不能确认空/满 用size和array.length

