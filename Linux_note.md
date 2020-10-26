

# Linux Note 



## A

## B

## C

#### 1.1 chown

更改文件的所有者

sudo chown    root           isce.log

​                  目的所有者   要更改的文件





## D

## E



## F



## G



## H

## I

## J

## K

## L

ls -s

所列的长格式目录，第一行的字母含义：

|  -   |   普通文件   |
| :--: | :----------: |
|  d   |     目录     |
|  c   | 字符设备文件 |
|  b   |  块设备文件  |
|  l   | 符号链接文件 |
|      |              |



## M

## N

## O

## p

根目录下，/proc，系统自动产生的映射，该目录的文件记录着有关系统硬件运行的信息。



## Q

## R

#### 1.1 Regular Expression

正则表达式



#### 1.2 rmdir

删除空的子目录，

rmdir -p

递归删除空的子目录，如果父目录也为空，最后也删除。



## S

### shutdown

~~~shell
shutdown -h now
shutdown +10
~~~



halt = shutdown -h

poweroff = halt -p

shutdown -r = reboot



### Shell



### Skel

新建用户时，root将把/etc/skel目录下的文件，全部copy到新用户的根目录。



## T

## U

## V

## W

## X

## Y

## Z











