create a new repository on the command line

git init
git config --global core.autocrlf false
git add *.*
git commit -m "first commit"
git remote add origin https://github.com/lamer0712/eurecar.git
git push -u origin master

push an existing repository from the command line
git status
git add *.*
git commit -m "2nd commit"   	% git commit
git push -u origin master	% git push


http://blog.sz21c.com/550
git branch mbtest1 
git push origin mbtest1 	% mbtest1 ������ Ǫ��
git checkout mbtest1  		% �۾� ���� ����

git checkout master
git merge mbtest1    		% master�� mbtest1 ��ü (master<-- mbtest1)

git branch -d mbtest1    	% mbtest1����

ignore
��Ʈ ������ 
.gitignore
���� ���� ���뿡 
*********************************************************************************************************
# Ȯ���ڰ� .a�� ���� ����
*.a

# �� ���ο��� Ȯ���ڰ� .a�� ������ �����ϰ� ������ lib.a�� �������� ����
!lib.a

# ���� ���丮�� �ִ� TODO������ �����ϰ� subdir/TODOó�� �������丮�� �ִ� ������ �������� ����.
/TODO

# .svn/ ���丮�� �ִ� ��� ������ ����
.svn/

# doc/notes.txt ������ �����ϰ� doc/server/arch.txt ������ �������� ����
doc/*.txt

# doc ���丮 �Ʒ��� ��� .txt ������ ����
doc/**/*.tx
*********************************************************************************************************