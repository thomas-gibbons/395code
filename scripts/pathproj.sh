# make sure to escape forward slash with backslash
# ie. foo\/bar\/395code

USER_PATH="<path to your 395code folder>" 


sed -i -e "s/.*HeaderPath=C.*/HeaderPath=${USER_PATH}\/$1\/Inc/" ./$1/.mxproject
sed -i -e "s/.*SourcePath=C.*/SourcePath=${USER_PATH}\/$1\/Src/" ./$1/.mxproject
