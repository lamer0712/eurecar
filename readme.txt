create a new repository on the command line

echo "# temp" >> README.md
git init
git config --global core.autocrlf false
git add *.*
git commit -m "first commit"
git remote add origin https://github.com/lamer0712/eurecar.git
git push -u origin master

push an existing repository from the command line

git remote add origin https://github.com/lamer0712/eurecar.git
git push -u origin master