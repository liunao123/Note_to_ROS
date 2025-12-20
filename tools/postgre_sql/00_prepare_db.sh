# 
echo "步骤 1/3: 创建 gtxc_db 数据库..."
createdb -U tyjt gtxc_db

echo "步骤 2/3: 安装 PostGIS 扩展..."
psql -U tyjt -d gtxc_db -c "CREATE EXTENSION postgis;"

# # 如果需要编译 PostGIS，可以使用以下命令（假设已下载并解压源码到 /data/postgresql/postgis-3.7.0dev）
# cd /data/postgresql/postgis-3.7.0dev
# export PATH=$PATH:/usr/bin ; ./configure --without-protobuf --with-pgconfig=/usr/bin/pg_config


echo "步骤 3/3: 创建数据库表结构..."
psql -U tyjt -d gtxc_db -f 00_mapping_system_schema.sql
psql  -d gtxc_db -f 00_mapping_system_schema.sql

# echo "数据库创建完成！"

# 方法1: 使用 -l 参数（需要指定已存在的数据库）
# psql -U tyjt -d gtxc_db -l

# 方法2: 连接后使用 \l 命令
# psql -U tyjt -d gtxc_db
# \l

python 04_export_poses.py -o /mnt/nvme0n1p2/project/postgresql/export/ --lat 31.41033324 --lon 120.65582103 --distance 100
# sudo -u postgres createuser tyjt -d -P

# 删除
#  sudo -u jmsroot dropdb gtxc_db
sudo -u jmsroot psql -c "\l+" # 验证删除