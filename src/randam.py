import random
import numpy as np

random.seed(10)
# 学习率,可以理解为纠正错误,或者逼近正确值的速度
lr = 0.001
# 每一次训练样本的个数
batch_size = 10


# 生成房子的面积数据200个
area_sample_data =  np.array([ random.randint(50, 200) for i in range(200)])
print(area_sample_data)

# 对应房子每个平方的真实价格，最后需要让机器猜到这个值，或者接近这个值
true_unit_preice = np.array([ random.uniform(1, 2) for i in range(200)] )

# 每个房子的总价格
sum_preice_label = area_sample_data * true_unit_preice

# 初始化第一次机器猜测的房子价格,最后我们要求的就是这个,让这个值接近上面的true_unit_preice
logic_unit_preice = random.uniform(-5,5)

# 求得样本数据以及标签的长度
area_sample_data_len = area_sample_data.shape[0]
print(area_sample_data_len)
sum_preice_label_len = sum_preice_label.shape[0]
print(sum_preice_label_len)

# 表示把数据训练完，需要猜测多少次,向下取整数 
batch_num_sum = area_sample_data_len // batch_size
print("batch_num_sum",batch_num_sum)#20

# 获得10个训练样本和标签
def get_train_label(num):
    cur_batch_num = num %batch_num_sum
    # print("cur_batch_num",cur_batch_num)
    return area_sample_data[cur_batch_num*batch_size: (cur_batch_num+1)*batch_size],\
           sum_preice_label[cur_batch_num*batch_size: (cur_batch_num+1)*batch_size]


# 这里的3000代表的是猜测的次数，可以看作猜测3000
for i in range(3000):
    # 每次去出10个样本进行进行训练迭代
    batch_sample_data, batch_preice_label =  get_train_label(i)
    print(batch_sample_data, batch_preice_label.shape)

    # 让机器进行预测，得到逻辑推理的房子总价
    logic_sum_preice = batch_sample_data * logic_unit_preice
    # 计算出10个样本平局的错误值
    loss =  np.mean(logic_sum_preice - batch_preice_label)

    # 如果预测房价高了
    if loss > 0:
        logic_unit_preice -= lr

    # 如果预测房价低了
    if loss < 0:
        logic_unit_preice += lr
    if i%100 == 0:
        print("loss:",loss)

# 最后打印机器猜到的数值
print('logic_unit_preice', logic_unit_preice)
