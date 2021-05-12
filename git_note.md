echo "# pure_locate" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/liunao123/pure_locate.git
git push -u origin main

…or push an existing repository from the command line
git remote add origin https://github.com/liunao123/pure_locate.git
git branch -M main
git push -u origin main



git submodule使用

~~~
git clone git@gitee.com:sagetech/c20009.git
cd c20009/
git status
git checkout library_robot_baotu 
git submodule update
git submodule init
git submodule update
cd src
~~~



将当前有修改的文件提交

~~~
git status：查看当前工作区的状态，是否有修改删除

git add <file>：将工作区某个文件的修改添加到暂存区（使用*时则将所有文件的修改都添加到暂存区）

git commit -m <describe>：将暂存区的修改提交到当前分支，并使用一段描述来记录这次提交

git push：将内容提交到远程仓库
~~~





查看当前所在在远程仓库

~~~
git remote -v
~~~



查看当前所在的分支

~~~
git branch -v
~~~

