# make sure to escape forward slash with backslash
# ie. foo\/bar\/395code

OBFS_PATH="<OBFS_PATH>"
OBFS_ENVHASH="<OBFS_ENVHASH>"

sed -i -e "s/.*HeaderPath=.*$1.*/HeaderPath=${OBFS_PATH}\/$1\/Inc/" ./$1/.mxproject
sed -i -e "s/.*SourcePath=.*$1.*/SourcePath=${OBFS_PATH}\/$1\/Src/" ./$1/.mxproject
sed -i -e "s/.*env-hash=.*/env-hash=${OBFS_ENVHASH}\/$1\/Src/" ./$1/.settings/language.settings.xml
