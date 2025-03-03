# Blog: https://zhuanlan.zhihu.com/p/551404912
# Docs: https://pypi.org/project/AllanTools/
# Blog: https://blog.csdn.net/m0_37605642/article/details/132635448

# import rosbag
# import math
# import numpy as np
# import matplotlib.pyplot as plt
# import allantools
#
# # 设置 ROS bag 文件路径和 IMU 数据话题
# bag_file_path = "/home/kino/Downloads/rosbag_2023_12_18-14_28_28.bag"
# imu_topic = "/imu"
#
# # 初始化存储数据的列表
# timestamps = []
# accel_data = []
# gyro_data = []
#
# # 读取 bag 文件
# with rosbag.Bag(bag_file_path, "r") as bag:
#     for topic, msg, t in bag.read_messages(topics=[imu_topic]):
#         # 假设 msg 具有加速度和陀螺仪数据
#         accel_data.append(
#             [
#                 msg.linear_acceleration.x,
#                 msg.linear_acceleration.y,
#                 msg.linear_acceleration.z,
#             ]
#         )
#         gyro_data.append(
#             [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
#         )
#         timestamps.append(msg.header.stamp.to_sec())
#
# # 转换为 numpy 数组
# accel_data = np.array(accel_data)
# gyro_data = np.array(gyro_data)
# timestamps = np.array(timestamps)
#
# print("Accelerometer Data Shape:", accel_data.shape)
# print("Gyroscope Data Shape:", gyro_data.shape)
#
# total_time = timestamps[-1] - timestamps[0]
# imu_rate = math.floor(len(timestamps) / total_time)
# print("IMU Rate: {} Hz".format(imu_rate))
#
# # 创建 Dataset 对象并计算 Allan 方差
# # 加速度计
# accel_dataset = allantools.Dataset(data=accel_data[:, 2], rate=imu_rate)
# accel_results = accel_dataset.compute("theo1")
#
# # 陀螺仪
# gyro_dataset = allantools.Dataset(data=gyro_data[:, 2], rate=imu_rate)
# gyro_results = gyro_dataset.compute("theo1")
#
# # 可视化 Allan 方差
# plt.figure(figsize=(12, 6))
#
# # 加速度计 Allan 方差
# plt.subplot(1, 2, 1)
# plt.loglog(
#     accel_results["taus"], accel_results["stat"], label="Allan Variance", color="blue"
# )
# plt.title("Allan Variance of Accelerometer Data")
# plt.xlabel("Averaging Time (s)")
# plt.ylabel("Allan Variance")
# plt.grid()
#
# # 找到 1 秒时的值并标注
# tau_index = np.argmin(np.abs(accel_results["taus"] - 1))  # 找到与1秒最接近的索引
# if tau_index < len(accel_results["stat"]):
#     plt.annotate(
#         f'{accel_results["stat"][tau_index]:.2e}',
#         xy=(accel_results["taus"][tau_index], accel_results["stat"][tau_index]),
#         xytext=(1.1, accel_results["stat"][tau_index] * 10),  # 标注位置
#         arrowprops=dict(facecolor="black", arrowstyle="->"),
#         fontsize=10,
#     )
#
# # 陀螺仪 Allan 方差
# plt.subplot(1, 2, 2)
# plt.loglog(
#     gyro_results["taus"], gyro_results["stat"], label="Allan Variance", color="orange"
# )
# plt.title("Allan Variance of Gyroscope Data")
# plt.xlabel("Averaging Time (s)")
# plt.ylabel("Allan Variance")
# plt.grid()
#
# # 找到 1 秒时的值并标注
# tau_index_gyro = np.argmin(np.abs(gyro_results["taus"] - 1))  # 找到与1秒最接近的索引
# if tau_index_gyro < len(gyro_results["stat"]):
#     plt.annotate(
#         f'{gyro_results["stat"][tau_index_gyro]:.2e}',
#         xy=(gyro_results["taus"][tau_index_gyro], gyro_results["stat"][tau_index_gyro]),
#         xytext=(1.1, gyro_results["stat"][tau_index_gyro] * 10),  # 标注位置
#         arrowprops=dict(facecolor="black", arrowstyle="->"),
#         fontsize=10,
#     )
#
# plt.tight_layout()
# plt.savefig("allan_variance.png")
# plt.show()
#
#
#
#!/usr/bin/env python
import rospy
import sys
import allantools
import rosbag
import numpy as np
import csv
import rospkg
import os
import matplotlib.pyplot as plt  # only for plotting, not required for calculations
import math


def getRandomWalkSegment(tau, sigma):

    m = -0.5  # slope of random walk
    """""" """""" """""" """""" """""" """
    " Find point where slope = -0.5 "
    """ """""" """""" """""" """""" """"""
    randomWalk = None
    i = 1
    idx = 1
    mindiff = 999
    logTau = -999
    while logTau < 0:
        logTau = math.log(tau[i], 10)
        logSigma = math.log(sigma[i], 10)
        prevLogTau = math.log(tau[i - 1], 10)
        prevLogSigma = math.log(sigma[i - 1], 10)
        slope = (logSigma - prevLogSigma) / (logTau - prevLogTau)  # 随机漫步的斜率
        diff = abs(slope - m)  # 当前斜率与目标斜率的差值
        if diff < mindiff:
            mindiff = diff  # 更新最小差值
            idx = i
        i = i + 1

    """""" """""" """""" """""" """"""
    " Project line to tau = 10^0 "
    """""" """""" """""" """""" """"""
    x1 = math.log(tau[idx], 10)  # 当前点的横坐标
    y1 = math.log(sigma[idx], 10)  # 当前点的纵坐标
    x2 = 0
    y2 = m * (x2 - x1) + y1

    return (pow(10, x1), pow(10, y1), pow(10, x2), pow(10, y2))


def getBiasInstabilityPoint(tau, sigma):
    i = 1
    while i < tau.size:
        if (tau[i] > 1) and (
            (sigma[i] - sigma[i - 1]) > 0
        ):  # only check for tau > 10^0
            break
        i = i + 1
    return (tau[i], sigma[i])


def main(args):

    rospy.init_node("allan_variance_node")

    t0 = rospy.get_time()

    """""" """""" ""
    " Parameters "
    """""" """""" ""
    bagfile = rospy.get_param(
        "~bagfile_path", "/home/kino/Downloads/rosbag_2023_12_17-16_09_44.bag"
    )  # 输入的bag文件路径
    topic = rospy.get_param("~imu_topic_name", "/imu")  # 输入的imu topic名称
    axis = rospy.get_param("~axis", 0)  # 输入的轴，0为x轴，1为y轴，2为z轴
    sampleRate = rospy.get_param("~sample_rate", 200)  # 输入的采样频率
    isDeltaType = rospy.get_param(
        "~delta_measurement", False
    )  # 是否是delta measurement
    numTau = rospy.get_param("~number_of_lags", 2218400)  # 输入的tau数目
    resultsPath = rospy.get_param("~results_directory_path", "/tmp/")  # 输出的结果路径

    """""" """""" """""" """""" ""
    " Results Directory Path "
    """""" """""" """""" """""" ""
    if resultsPath is None:
        paths = rospkg.get_ros_paths()
        path = paths[1]  # path to workspace's devel
        idx = path.find("ws/")  # 找到路径
        workspacePath = path[0 : (idx + 3)]  # 取到workspace的路径
        resultsPath = workspacePath + "av_results/"  # 结果输出路径

        if not os.path.isdir(resultsPath):
            os.mkdir(resultsPath)

    print(
        "\nResults will be save in the following directory: \n\n\t %s\n" % resultsPath
    )

    """""" """""" """"""
    " Form Tau Array "
    """""" """""" """"""
    taus = [None] * numTau  # 初始化tau数组

    cnt = 0
    for i in np.linspace(
        -2.0, 5.0, num=numTau
    ):  # lags will span from 10^-2 to 10^5, log spaced
        taus[cnt] = pow(10, i)  # 将tau数组赋值,维度在10^-2 到 10^5
        cnt = cnt + 1

    """""" """""" """""
    " Parse Bagfile "
    """ """""" """""" ""
    bag = rosbag.Bag(bagfile)

    N = bag.get_message_count(topic)  # 在bag文件中找到该topic的消息数量

    data = np.zeros((6, N))  # 初始化数据矩阵，维度为6*N

    if isDeltaType:
        scale = sampleRate
    else:
        scale = 1.0

    cnt = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):  # 遍历bag文件中的消息
        data[0, cnt] = msg.linear_acceleration.x * scale
        data[1, cnt] = msg.linear_acceleration.y * scale
        data[2, cnt] = msg.linear_acceleration.z * scale
        data[3, cnt] = msg.angular_velocity.x * scale
        data[4, cnt] = msg.angular_velocity.y * scale
        data[5, cnt] = msg.angular_velocity.z * scale
        cnt = cnt + 1

    bag.close()

    print("[%0.2f seconds] Bagfile parsed\n" % (rospy.get_time() - t0))

    """""" """""" """"""
    " Allan Variance "
    """""" """""" """"""
    if axis == 0:
        currentAxis = 1  # 循环通过所有轴1-6
    else:
        currentAxis = axis  # 只需循环一次，然后中断

    while currentAxis <= 6:
        # taus_used 对应了是否可以使用taus的数组，adev是allan deviation degree的缩写，为allan偏差；adev_err是allan偏差的误差；adev_norm是allan偏差的标准化;
        (taus_used, adev, adev_err, adev_n) = allantools.oadev(
            data[currentAxis - 1],
            data_type="freq",
            rate=float(sampleRate),
            taus=np.array(taus),
        )  # 计算allan variance

        randomWalkSegment = getRandomWalkSegment(
            taus_used, adev
        )  # 计算random walk segment
        biasInstabilityPoint = getBiasInstabilityPoint(
            taus_used, adev
        )  # 计算bias instability point

        randomWalk = randomWalkSegment[3]  # 获取random walk segment的纵坐标
        biasInstability = biasInstabilityPoint[1]  # 获取bias instability point的纵坐标

        """""" """""" """
        " Save as CSV "
        """ """""" """"""
        if currentAxis == 1:
            fname = "allan_accel_x"
            title = "Allan Deviation: Accelerometer X"
        elif currentAxis == 2:
            fname = "allan_accel_y"
            title = "Allan Deviation: Accelerometer Y"
        elif currentAxis == 3:
            fname = "allan_accel_z"
            title = "Allan Deviation: Accelerometer Z"
        elif currentAxis == 4:
            fname = "allan_gyro_x"
            title = "Allan Deviation: Gyroscope X"
        elif currentAxis == 5:
            fname = "allan_gyro_y"
            title = "Allan Deviation: Gyroscope Y"
        elif currentAxis == 6:
            fname = "allan_gyro_z"
            title = "Allan Deviation: Gyroscope Z"

        print(
            "[%0.2f seconds] Finished calculating allan variance - writing results to %s"
            % (rospy.get_time() - t0, fname)
        )

        f = open(resultsPath + fname + ".csv", "wt")

        try:
            writer = csv.writer(f)
            writer.writerow(("Random Walk", "Bias Instability"))
            writer.writerow((randomWalk, biasInstability))
            writer.writerow(("Tau", "AllanDev", "AllanDevError", "AllanDevN"))
            for i in range(taus_used.size):
                writer.writerow((taus_used[i], adev[i], adev_err[i], adev_n[i]))
        finally:
            f.close()

        """""" """""" """
        " Plot Result "
        """ """""" """"""
        plt.figure(figsize=(12, 8))
        ax = plt.gca()
        ax.set_yscale("log")
        ax.set_xscale("log")

        plt.plot(taus_used, adev)
        plt.plot(
            [randomWalkSegment[0], randomWalkSegment[2]],
            [randomWalkSegment[1], randomWalkSegment[3]],
            "k--",
        )
        plt.plot(1, randomWalk, "rx", markeredgewidth=2.5, markersize=14.0)
        plt.plot(biasInstabilityPoint[0], biasInstabilityPoint[1], "ro")

        plt.grid(True, which="both")
        plt.title(title)
        plt.xlabel("Tau (s)")
        plt.ylabel("ADEV")

        for item in (
            [ax.title, ax.xaxis.label, ax.yaxis.label]
            + ax.get_xticklabels()
            + ax.get_yticklabels()
        ):
            item.set_fontsize(20)

        plt.show(block=False)

        plt.savefig(resultsPath + fname)

        currentAxis = (
            currentAxis + 1 + axis * 6
        )  # increment currentAxis also break if axis is not =0

    inp = input("Press Enter key to close figures and end program\n")


if __name__ == "__main__":
    main(sys.argv)
