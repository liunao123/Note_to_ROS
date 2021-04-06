



## 构造函数

~~~
inline String::String(const char* cstr = 0)
{
if(cstr)
{
   m_data = new char[strlen(cstr) +1 ];
   strcpy(m_data,cstr);

}



}

~~~





## 1 copy

### 拷贝构造

~~~c++
String(const String& str);
~~~





### 拷贝赋值-copy assign

~~~c++
String& operator=(const String& str);
~~~







<u>NOTE：</u>

class里面有指针就要自己写，copy--

编译器默认copy的是指针，这样就是新的变量，其实也是一个指针，而且与旧的指向同一个内存，并没有真正的创建出一块内存存放变量。(浅copy)







## 析构函数

~String

清理作用,clear up



class里面有指针，多半是涉及内存的动态分配，

在对象死亡之前一刻（离开变量所在的scope，作用域），析构函数会被都用，在此将该对象杀掉，释放系统内存。



~~~c++
inline String::~String()
{
delete[] m_data;
}
~~~



## copy assignment operator

重写String的=操作符。

~~~c++
String a;
string b;
b = a;
~~~





把a拷贝到b里面，steps:

<u>0。自我赋值检测</u>

1.清空b，delete b；

2.创建一个和a一样大小的对象，（分配了复制了所需的空间）

3。赋值。



~~~c++
inline String& String::operator=(const String& str)
{
	if(this == &str)//检测自我赋值-self assignment-非常重要
		return *this;
	//按以上的步骤，写
	
	
	

}
~~~



# 20210217-HJ-16-vector深度搜索



# 20210217-HJ-17

![image-20210217100456608](/home/xiaofan/.config/Typora/typora-user-images/image-20210217100456608.png)



# 20210217-HJ-18

## deque



![Screenshot from 2021-02-17 10-22-06](/home/xiaofan/Pictures/Screenshot from 2021-02-17 10-22-06.png)













