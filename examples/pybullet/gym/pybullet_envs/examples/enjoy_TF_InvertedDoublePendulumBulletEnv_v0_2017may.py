#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import gym
import numpy as np
import pybullet as p
import pybullet_envs
import time

def relu(x):
    return np.maximum(x, 0)

class SmallReactivePolicy:
    "Simple multi-layer perceptron policy, no internal state"
    def __init__(self, observation_space, action_space):
        assert weights_dense1_w.shape == (observation_space.shape[0], 64.0)
        assert weights_dense2_w.shape == (64.0, 32.0)
        assert weights_final_w.shape  == (32.0, action_space.shape[0])

    def act(self, ob):
        x = ob
        x = relu(np.dot(x, weights_dense1_w) + weights_dense1_b)
        x = relu(np.dot(x, weights_dense2_w) + weights_dense2_b)
        x = np.dot(x, weights_final_w) + weights_final_b
        return x

def demo_run():
    env = gym.make("InvertedDoublePendulumBulletEnv-v0")

    cid = p.connect(p.GUI)

    pi = SmallReactivePolicy(env.observation_space, env.action_space)

    while 1:
        frame = 0
        score = 0
        restart_delay = 0
        obs = env.reset()

        while 1:
            time.sleep(0.05)
            a = pi.act(obs)
            obs, r, done, _ = env.step(a)
            score += r
            frame += 1
            still_open = env.render("human")
            if still_open==False:
                return
            if not done: continue
            if restart_delay==0:
                print("score=%0.2f in %i frames" % (score, frame))
                restart_delay = 60*2  # 2 sec at 60 fps
            else:
                restart_delay -= 1
                if restart_delay > 0: continue
                break

weights_dense1_w = np.array([
[ -0.5857, +0.1810, +0.2839, +0.1278, -0.4302, -0.3152, +0.5916, -0.0635, +0.6259, +0.2873, -0.0572, -0.3538, -0.8121, +0.2707, +0.1656, -0.2103, -0.1614, -0.2789, -0.5856, -0.4733, +0.1838, +0.1063, +0.7629, +0.0873, +0.1480, +0.1768, +0.6522, +0.1158, -0.0816, +0.6542, -0.8870, +0.1775, +0.1532, +0.2268, -0.0313, -0.0470, +0.5328, -0.0570, +0.4820, -0.3772, -0.7581, +0.2835, -0.3566, +0.9371, -0.0441, -0.1797, -0.2859, -0.0238, +0.0261, -0.0296, -0.1406, +0.2869, +0.1279, +0.6653, +0.5643, -0.3136, +0.7751, +0.2341, +0.1903, +0.8283, -0.0697, +0.1276, -0.0250, -0.0053],
[ +0.3741, +0.4844, -0.0638, -0.3205, +0.3137, +0.9636, +0.5329, +0.6882, +0.2983, -0.6675, -0.6372, +0.2065, -0.2645, -0.4789, +0.2326, -0.0691, -0.5905, -0.3354, +0.3428, +0.4253, +0.9111, -0.4751, -0.2124, +0.3920, +0.2897, -1.1101, +0.1894, -0.4025, -0.1125, -0.0627, +0.2347, -0.8787, +0.1019, +0.9128, +0.2544, -0.3933, +0.6485, -0.1936, -0.2402, +0.5012, -0.0918, +0.3160, -0.7860, +0.3439, -0.4268, -0.1788, -0.3930, +0.5128, +0.2338, +0.2571, +0.1343, +0.9850, -0.7074, +0.3532, +0.3048, -0.4542, +0.5539, -0.4409, -0.2003, -0.4837, -0.3554, -0.4447, -0.0817, -0.8497],
[ +0.0825, +0.5847, +0.4837, +0.5144, +0.4770, +0.0199, +0.4275, -0.4530, +0.8499, -0.2840, +0.3817, -0.5098, -0.2155, -0.1475, +0.1145, -0.1871, -0.0526, +0.3583, -0.3537, -0.7111, -0.6116, +0.3406, -0.6360, +0.7786, +0.6628, -0.0493, +0.3505, -0.0376, -0.6556, +1.0748, -0.5329, +0.6477, -0.7117, +0.3723, +0.6006, +0.0171, +0.0012, +0.4910, -0.5651, -0.6868, +0.2403, +0.0254, -0.4416, +0.7534, -0.0138, -1.1298, +0.5447, +0.0974, +0.1988, -0.2161, -0.3126, -0.5731, -0.1278, +0.2995, -0.1200, -0.7917, +0.5326, +0.4562, -0.0144, +0.5717, -0.4065, +0.1494, +0.7100, +0.2461],
[ -0.2861, +0.4314, -0.2982, -0.1401, -0.1033, +0.5287, -0.6620, -0.3975, +0.0038, +0.1991, -0.7079, -0.9000, +0.1659, +0.3623, -0.0752, -0.1907, -0.2335, -0.5143, +0.2324, -0.0487, +0.1583, -0.5989, +0.5957, +0.2150, -0.0335, +0.2154, +0.3279, -0.7976, +0.5320, -0.4438, +0.2170, -0.3841, -0.0039, -0.0847, -0.0028, -0.4278, -0.2393, -0.9239, +0.2880, -0.1437, -0.0941, -0.0796, -0.3906, -0.3224, +0.1038, -0.1929, -0.2713, -0.4157, -0.2178, +0.5729, -0.2065, +0.0059, +0.3879, +0.0590, +0.1759, +0.0677, -0.0170, -0.2252, +0.3301, -0.0599, +0.3791, -0.1455, +0.2200, -0.0499],
[ -0.4403, +0.7182, +0.7941, +0.1469, +1.5777, +0.3426, +0.0923, +0.2160, +1.1492, -0.5206, -0.2659, -0.1504, +0.2739, -1.3939, +0.8992, -1.1433, -0.3828, -0.2497, -0.2172, +0.2318, -0.3605, +0.6413, -1.9095, +1.4785, -0.1274, -0.7208, -0.0802, -0.8779, -1.6260, +0.9151, +0.8289, -0.0902, -0.3551, +0.6198, +1.7488, +0.0739, -1.2022, -0.3536, -1.5187, +0.1839, +1.4258, +0.4217, +0.1503, -0.0460, +0.2327, -0.4139, -0.3668, +0.2997, +0.6856, +0.6917, -0.3856, -0.3620, +0.1578, -0.8349, -1.0796, -0.0319, -1.1966, -0.8122, +0.5053, -0.5033, -0.9207, -0.1193, -0.7625, +0.1379],
[ -0.0321, -0.3206, -0.4516, +0.3420, +1.0964, +0.0311, +0.4654, -0.2367, +0.3347, -0.2798, -0.8169, -0.1555, +0.9397, -0.5597, +0.7113, -0.3642, -0.2840, -0.1323, -0.1000, +0.2283, +0.3612, -0.4784, +0.0504, +0.5310, -0.0887, +0.2926, +0.5069, -0.5645, -0.0976, -0.2594, +0.4425, +0.9223, -0.5637, -0.2336, -0.1316, -0.6564, -0.2780, -0.2409, -0.1637, +0.4506, +0.7018, -0.1299, +0.7172, +0.1207, +0.4375, +0.3836, +0.2781, -0.7792, -0.5317, +0.4510, +0.2423, -0.0588, -0.4254, -0.6381, -0.8205, +0.6417, +0.1904, -0.2618, +0.5900, -0.3899, -0.7851, -0.4769, -0.3688, -0.3510],
[ -0.8366, -0.3157, -0.1130, +0.2005, +0.3713, -0.4351, -0.1278, -0.5689, +0.3229, -0.5981, -0.4917, -0.4160, -0.5504, +0.2225, -0.1581, -0.6457, +0.1001, -1.0635, +0.2368, +0.2494, -0.4054, -0.1699, -0.1316, +0.2614, +0.3016, +0.4222, -0.1548, -0.0766, -0.5226, -0.3576, -0.2433, -0.5495, +0.0056, +0.0193, +0.2353, +0.3986, +0.3580, -0.7886, +0.3928, +0.1831, +0.4319, +0.2276, -0.3062, +0.0413, -0.4288, +0.1365, +0.3176, +0.3564, +0.5668, -0.4401, -0.9576, -0.1435, +0.0304, -0.5575, +0.0412, -0.1096, +0.2207, +0.1227, -0.0051, +0.5808, -0.1331, +0.1368, +0.4170, -0.8095],
[ -0.6368, -1.3221, -0.4492, -1.5414, +0.4004, -2.8780, -0.1748, -0.8166, +1.7066, +1.0714, -0.4755, +0.3020, +0.0422, +0.3466, +0.4472, -0.6209, -3.3768, -0.0806, +1.3624, -2.4155, +1.0886, +0.3412, +0.0891, +1.6821, -0.5361, +0.3952, +1.5120, +0.3910, +1.9500, -0.9065, -1.3452, +0.0904, -0.0389, +0.2817, -1.8375, +0.8131, -1.5287, +0.3115, +1.4069, -0.3424, +1.6101, +2.6775, +0.5516, +1.6500, -0.4138, -0.0170, +1.0008, -0.7865, +0.0551, +2.2068, -0.0108, +0.3207, -1.1884, +0.3792, -0.6435, +0.2858, -0.6881, +0.1554, -1.6926, -0.0975, -1.4120, -0.0827, -1.5186, +0.2526],
[ -0.2900, -0.2805, +0.9182, -0.8893, +0.7345, -0.9015, -0.2696, +0.2344, +0.3889, +0.6790, +0.3657, -0.1995, -0.6738, -0.4166, +0.1690, -0.3798, -0.9872, -0.2558, -0.4205, -0.6190, -0.0092, -0.2261, -0.2738, +0.2977, -0.7348, +0.4872, +0.4776, -0.1364, +0.5836, -0.2688, -0.4261, -0.3612, -0.3533, +0.4665, +0.0155, +1.0116, -0.7139, -0.3707, -0.4429, -0.0383, +0.6716, +0.5972, +0.3506, +0.3294, -1.3734, -0.5905, -0.1168, -0.2609, +0.3436, +0.8277, +0.4965, +0.3005, -0.2929, +0.1501, -0.2655, +0.3860, -0.3946, +0.8764, +0.7927, +0.0541, -1.0912, -0.2006, -0.6928, +0.4653]
])

weights_dense1_b = np.array([ -0.1146, +0.2897, +0.0137, +0.0822, +0.0367, +0.0951, -0.0657, +0.0653, -0.0729, -0.0501, -0.6380, -0.4403, +0.0660, +0.0693, -0.4353, -0.2766, -0.1258, -0.6947, -0.1616, -0.0453, +0.1498, -0.2340, -0.0764, +0.2020, +0.4812, +0.0908, -0.1883, -0.0753, -0.0373, -0.4172, -0.1071, +0.0861, -0.1550, -0.0648, -0.1473, +0.1507, -0.0121, -0.5468, -0.1529, -0.3341, +0.0239, -0.0463, -0.0044, -0.0541, +0.0384, +0.3028, +0.3378, +0.0965, +0.0740, +0.1948, -0.1655, +0.1558, +0.1367, -0.1514, +0.0540, -0.0015, -0.1256, +0.3402, -0.0273, -0.2239, -0.0073, -0.6246, +0.0755, -0.2002])

weights_dense2_w = np.array([
[ +0.5019, +0.3831, +0.6726, +0.3767, +0.2021, -0.1615, +0.3882, -0.0487, -0.2713, +0.1173, -0.2218, +0.0598, +0.0819, -0.1157, +0.5879, -0.3587, +0.1376, -0.2595, +0.0257, -0.1182, +0.0723, +0.5612, -0.4087, -0.4651, +0.0631, +0.1786, +0.1206, +0.4791, +0.5922, -0.4444, +0.3446, -0.0464],
[ -0.0485, +0.0739, -0.6915, +0.5446, -0.2461, +0.1557, +0.8993, -0.7537, +0.1149, +0.0575, -0.1714, -0.3796, +0.3508, -0.2315, +0.4389, -1.4456, -1.3490, -0.1598, -1.0354, -0.2320, -0.3765, +0.1070, -0.7107, +0.4150, +0.2711, -0.2915, -0.7957, +0.7753, -0.0425, -0.1352, +0.3018, -0.0069],
[ -0.4047, +1.0040, -0.4570, +0.3017, +0.1477, -0.0163, +0.4087, -0.6368, -0.0764, -0.0695, +0.0208, -0.2411, +0.1936, +0.0047, +0.0107, -0.8538, -0.5887, -0.0524, -1.4902, +0.2858, +0.4396, -0.3433, -0.6778, -0.7137, +0.4587, +0.3359, -0.7350, -1.0813, -0.1296, +0.1748, -0.3830, -0.2103],
[ +0.0503, -0.3342, -0.6057, +0.2217, +0.3164, -0.1881, -0.5867, -0.2471, -0.2527, -0.0444, +0.1874, -0.0960, +0.2039, -0.0488, +0.1741, -0.1623, -0.0758, -0.2354, -0.5986, -0.2129, -0.2470, +0.3317, -0.4795, -0.6380, +0.1494, +0.0115, -0.2746, -0.8428, -0.0118, -0.0604, +0.0886, -0.0408],
[ -0.1932, -1.3896, +0.3919, -0.4700, -0.9806, -0.1554, +0.3132, +0.4138, -0.4943, -0.1408, -0.0976, +0.1551, -0.0180, +0.0864, -0.0053, -0.2430, +0.4948, +0.2709, -0.3488, +0.2085, -0.2124, -0.3025, -0.0692, +0.3884, +0.5764, +0.5783, +0.4351, -0.2633, -0.9288, +0.2218, -0.9049, -0.2970],
[ -0.2841, -0.3393, -0.1062, -0.1415, +0.0257, +0.0816, -0.4833, -0.2775, +0.0308, -0.0344, +0.5451, +0.1588, -0.7454, -0.1444, +0.4189, -0.2001, -2.0586, -0.0616, -1.4463, +0.0076, -0.7703, +0.3279, -0.7009, +0.6046, -0.1615, -0.5188, -0.7503, +0.0615, +0.1815, -0.2512, +0.0321, -0.1834],
[ +0.3751, +0.2932, -0.6815, +0.3771, +0.0603, -0.2035, -0.2644, -1.0120, -0.0661, -0.0846, +0.1209, +0.0367, +0.0493, -0.2603, -0.1608, -0.7580, -0.8609, +0.1415, -0.7626, -1.0209, -0.7498, -0.0732, -0.8138, -0.2292, +0.5803, -0.2718, -1.4684, -0.1584, +0.2096, +0.1336, +0.3632, +0.0216],
[ -0.0625, -0.1233, -0.2715, +0.5541, +0.3121, +0.0265, +0.4501, -1.1024, -0.1160, -0.1005, -0.0844, -0.0516, +0.0916, +0.0901, +0.3710, -0.5753, -0.3728, -0.1103, -0.6285, -0.2179, +0.1570, +0.1168, -0.9312, +0.0135, -0.0376, -0.1693, -0.5358, -0.0028, +0.2105, -0.7373, +0.2776, +0.2326],
[ -0.5378, -0.3201, +0.3606, +0.1331, +0.0120, -0.2421, -0.0230, +0.4622, -0.3140, +0.0803, -0.6897, -0.4704, +0.2685, +0.0803, -0.7654, -0.1433, +0.0242, +0.0917, +0.2458, +0.0457, -0.2503, -0.1197, +0.1454, -0.1523, -0.4095, +0.1856, +0.0678, -1.0011, +0.0117, +0.1789, -0.4063, -0.0888],
[ -0.6352, -0.6358, -0.2066, +0.0758, -0.2973, -0.3014, -0.0556, -0.0912, -0.2729, -0.1492, -0.1928, -1.8768, +0.2183, +0.0801, +0.1288, -1.2493, +0.1115, +0.2797, -0.1458, +0.0062, -0.0402, -0.8945, -0.2231, -0.1154, +0.3635, -0.3021, +0.1402, -0.7347, +0.2772, +0.3182, -0.9708, +0.0376],
[ +0.6899, +0.3471, -0.5863, +0.1497, +0.1616, -0.0497, +0.3579, -0.6421, +0.4529, -0.1588, +0.9250, +0.2357, -0.0712, -0.1793, -0.0231, -0.4814, -0.7654, +0.0591, -0.6866, -0.1705, +0.2255, -0.0007, -0.3890, +0.6114, +0.0443, -0.6929, -0.7734, +0.2314, -0.0231, -0.6386, +0.1237, +0.0472],
[ -0.2496, -0.1687, +0.1234, +0.4152, +0.4207, -0.1398, +0.1287, +0.5903, +0.0530, -0.1181, +0.0803, -0.0641, -0.1198, -0.4702, -0.3669, +0.2340, -0.3778, +0.4341, +0.2411, -0.2171, -0.3051, -0.2397, +0.1756, +0.4040, +0.0682, +0.1575, +0.4137, +0.0887, -0.1998, +0.2221, -0.2474, -0.0559],
[ -2.2466, -1.2725, +0.5947, -0.3192, -0.2665, -0.0129, -0.7615, +0.1148, +0.2745, -0.0556, -1.3313, -0.7143, -0.5119, -0.0572, -0.1892, -0.3294, -0.0187, -0.7696, +1.0413, +0.4226, +0.1378, -1.3668, +0.9806, -0.1810, -0.2307, -0.4924, +0.7163, -1.2529, -0.3216, +0.1683, -0.6657, -0.1121],
[ +0.1227, +0.2433, -0.1292, -0.7152, -0.1342, -0.1116, -0.2695, +0.0243, -0.0770, -0.1713, +0.2837, +0.2076, -0.7322, -0.1657, -0.3407, -0.4378, +0.0679, -0.3777, +0.3025, -0.6780, -0.2326, +0.1463, +0.0535, -0.6373, -0.2027, -0.5404, -0.1598, +0.1511, -0.1776, +0.0854, +0.1753, -0.0342],
[ -0.1772, -0.2654, -0.4170, -0.3301, +0.2956, -0.4234, +0.0835, +0.2869, -0.2804, -0.2073, -0.3291, -0.5897, -0.4116, -0.0447, +0.1601, +0.1602, +0.1691, -0.2014, -0.0502, +0.1167, -1.0103, -0.4297, -0.2039, -0.0859, +0.2756, -0.1768, -0.2726, -0.0256, -0.0834, +0.0852, +0.0930, -0.0606],
[ -0.5390, -0.5441, +0.3202, -0.1018, +0.0059, +0.1799, -0.1917, +0.3674, +0.2576, -0.0707, -0.4401, -0.3990, +0.0565, +0.0751, -0.5959, +0.3866, +0.2763, -0.2564, +0.4937, +0.5076, +0.3941, -0.3593, +0.4346, +0.2561, -0.0762, -0.2873, +0.6820, -0.3032, -0.3268, +0.1319, -0.3643, +0.0292],
[ +0.1816, -0.0451, -0.9370, +0.1335, -0.1030, -0.0400, +0.0311, -1.3751, -0.1860, +0.1559, +0.5395, +0.3994, -0.1703, -0.1157, +0.6342, -0.4726, -0.6213, -0.2096, -0.7549, -0.9815, -0.3798, +0.5286, -0.8413, +0.2577, +0.2223, -1.2260, -1.3571, -0.0970, +0.3334, -0.2096, +0.3566, -0.1703],
[ +0.0635, +0.1541, -0.2206, +0.0924, +0.1302, +0.1947, -0.3868, -0.6834, -0.0603, -0.3752, +0.3103, -0.1699, -0.0833, -0.1190, -0.0310, -0.5480, -1.1421, -0.0020, -0.3611, -0.3800, -0.0638, +0.0811, -0.5886, +0.0690, +0.1925, +0.0710, -0.3142, +0.1837, +0.2125, -0.1217, +0.2185, +0.0458],
[ -0.3973, +0.0486, +0.2518, -0.3208, +0.1218, -0.5324, -0.3417, +0.0322, -0.0088, +0.0214, +0.2725, +0.0960, -0.2949, -0.1770, -0.1511, +0.0259, +0.1161, -0.8829, +0.2415, +0.0939, -0.7213, +0.2220, +0.1687, -0.1802, -0.0539, +0.1786, +0.6638, +0.3559, +0.2343, +0.3212, +0.4396, -0.1385],
[ -0.2384, -0.5346, -0.2323, -0.2277, +0.3503, -0.0308, -0.2004, -0.1096, -0.2587, -0.1143, +0.2579, +0.2382, -0.5883, -0.1277, +0.2257, -0.0244, -0.9605, -0.4244, -0.7321, +0.3017, -1.6256, -0.2074, -0.8327, +0.0607, -0.0751, -0.0153, -0.4485, +0.1758, +0.1821, +0.2625, +0.0108, -0.2395],
[ -0.5639, -0.3613, +0.1291, -0.2132, +0.4927, -0.0604, -0.8067, +0.0933, -0.1483, -0.0321, -0.6843, -0.3064, -0.5646, -0.2040, -0.0414, +0.6092, +0.4903, -0.9346, +0.3389, +0.2040, -0.0295, -0.2196, +0.4264, +0.0312, -1.1801, +0.3008, +0.7719, +0.2140, -0.0257, +0.5275, -0.0553, +0.0362],
[ -0.6039, -1.2848, +0.6301, -0.1285, +0.2338, -0.2585, -0.3217, +0.4326, +0.0441, -0.0356, -0.5720, -0.8739, -0.3924, +0.2499, -0.2620, +0.1396, -0.0701, -0.2239, +0.2612, +0.1646, +0.7769, -0.6382, +0.8720, -0.1956, -0.1779, -0.1608, -0.0358, -0.4453, -0.1154, +0.5532, -0.9282, +0.0031],
[ -0.1990, +0.3708, -0.0049, -0.3260, -0.0465, +0.0415, +0.1601, +0.0019, +0.0114, +0.0438, +0.0893, +0.3056, -0.6166, +0.1145, -0.6742, +0.0483, +0.0739, -0.1139, +0.5772, -1.5569, +0.4253, -0.0769, +0.4014, -0.6817, +0.0228, -0.0383, -0.0844, -0.1560, +0.1414, -0.3420, +0.3664, -0.2293],
[ -0.0917, -0.8692, +0.4704, +0.1796, -0.1346, -0.5246, +0.0622, +0.3420, -0.5879, -0.0445, -0.3444, -0.0490, +0.0956, -0.0753, -0.8856, +0.1275, +0.1592, +0.3569, +0.1774, +0.2723, +0.1125, -0.1718, +0.2451, -0.0132, +0.1584, -0.0197, +0.0700, -0.2156, +0.0094, +0.4639, -0.6721, -0.2180],
[ +0.0578, -0.1570, -0.1623, -0.1359, +0.1346, +0.1821, -0.0696, -0.0570, +0.0011, +0.1216, +0.1069, -0.0841, +0.1017, -0.1663, -0.6005, -0.4583, -0.2606, -0.0292, +0.0321, -0.5614, -0.4416, +0.0355, +0.2081, +0.3517, +0.0619, -1.0007, -0.0765, +0.1769, -0.1286, +0.5833, -0.1758, -0.1957],
[ -0.0013, +0.3157, +0.0395, -1.0792, -0.1198, -0.2945, -0.0090, +0.3281, -0.0618, -0.0806, +0.0768, +0.2802, -0.2311, -0.2302, +0.0506, +0.0552, +0.3727, +0.3610, +0.2029, -0.1743, +0.4557, -0.1761, -0.5039, -0.9115, +0.2842, +0.1317, -0.5961, -0.4214, -1.0727, +0.3308, +0.2380, -0.3348],
[ +0.2455, -0.1299, +0.3117, -1.0169, -0.3417, +0.0310, -0.4793, +0.5334, -0.4799, -0.3291, -0.1344, +0.3732, -0.1514, +0.1574, -0.1819, -0.0206, +0.5675, -0.6992, +0.4815, -0.1497, -0.3804, +0.1389, +0.5850, -0.2920, +0.2569, -0.3527, +0.3641, -0.2014, -0.1457, +0.2365, -0.2335, -0.2610],
[ -0.2252, +0.1225, +0.0953, -0.0193, +0.3955, -0.0800, +0.0090, -0.4155, +0.1851, +0.3392, -0.3260, -0.3907, +0.1320, +0.1266, +0.0579, +0.1819, -0.5793, -0.2230, +0.1351, -0.1519, -0.0527, -0.0036, +0.1243, +0.1387, -0.2874, -0.4997, -0.3251, +0.0435, -0.5244, +0.1051, -0.2081, +0.2126],
[ -0.6574, +0.6789, +0.1026, -0.5191, +0.1058, -0.6812, +0.1798, -0.1029, +0.0757, -0.0089, +0.1539, +0.4688, -0.1306, +0.0595, -0.8136, -0.4843, +0.3675, +0.1800, +0.2641, -0.0589, +0.0613, +0.2019, -0.0765, -0.1216, -0.4588, +0.0629, +0.1133, +0.7055, -2.8075, +0.3867, +0.4791, -0.1118],
[ +0.2771, +0.3461, -0.8556, -0.0316, +0.3640, -0.1380, -0.3765, -0.9258, -0.0693, -0.1283, +0.0576, -0.0792, +0.4468, -0.5001, +0.5939, -1.2226, -0.9252, -0.3980, -1.3444, -0.9488, -0.7199, +0.4289, -1.8749, -0.0867, +0.3905, -0.4535, -0.5607, -0.2247, -0.0359, -0.4125, +0.7124, -0.1963],
[ -0.2584, -0.5358, -0.0916, +0.0765, +0.0615, -0.1887, -0.2253, -0.7855, -0.0061, -0.1887, +0.5511, +0.3207, -0.2055, -0.1694, +0.4772, -1.0356, -0.9884, -0.2620, -0.1214, +0.9733, -0.9700, -0.3205, -0.7005, -0.2960, +0.1132, -0.0352, +0.3491, -0.2440, +0.1108, +0.1083, +0.3029, -0.0031],
[ -0.6217, +0.1238, +0.0245, -0.1769, -0.2487, +0.0526, -0.0090, +0.1370, +0.2666, -0.0743, -0.8230, -0.7723, -0.0929, -0.1532, +0.6103, -0.4931, -1.3329, -0.3735, +0.0217, -0.1539, -0.4946, -1.0838, -0.5840, +0.1618, +0.2584, +0.4200, +0.1171, -0.5601, +0.1604, +0.0864, +0.2287, -0.0057],
[ -0.2220, +0.4837, -0.0825, +0.0143, +0.2734, -0.0853, +0.1578, -0.0112, +0.1829, +0.0390, +0.2151, -0.1538, -0.1111, -0.0773, +0.3439, -0.2134, -0.2884, -0.3831, +0.2767, -0.3149, +0.1463, +0.3230, +0.2187, -0.2309, -0.1096, +0.3709, -0.0105, +0.3709, +0.3034, -0.7602, +0.5988, -0.0595],
[ -0.6073, +0.1780, +0.1682, +0.1604, +0.3662, -0.0385, -0.1495, +0.3012, -0.2065, -0.0163, -1.0465, -0.8268, -0.0190, +0.0964, -0.2755, +0.0965, -0.3466, -0.3758, -0.1113, +0.1462, +0.3280, -0.1600, +0.1023, +0.1998, -0.3642, +0.2736, +0.3782, -0.2681, +0.2334, +0.1721, +0.0385, +0.0348],
[ -0.0582, -0.5750, +0.1279, +0.3630, -0.2404, -0.1511, +0.2650, -0.0324, -0.2258, +0.0007, +0.3051, -0.1875, -0.5106, +0.0104, +0.1335, -0.5282, -0.2210, +0.2648, -0.7506, +0.4975, -1.7048, +0.2378, -0.1771, +0.2981, +0.1252, +0.1384, -0.3384, -0.0830, +0.0966, +0.3728, -0.1980, -0.1953],
[ -1.0735, -0.2780, +0.1428, -0.0624, -0.0311, -0.2687, -0.1623, +0.2996, +0.1782, -0.1403, -0.3761, -1.3413, -0.2020, -0.0492, -0.6636, -0.2737, +0.2228, +0.3109, +0.1596, +0.0172, +0.1325, -1.4936, -0.0615, -0.1547, -0.2285, +0.2648, -0.1008, -1.6756, -0.2352, +0.0998, -0.4550, +0.2028],
[ -0.3866, -0.0107, +0.1052, +0.1423, +0.1160, +0.1712, -0.6206, -0.3505, -0.3298, -0.0362, +0.6768, +0.2086, -0.4348, -0.3577, +0.0131, -0.1640, +0.0160, -0.3891, -0.0180, -0.1064, -0.2494, +0.0340, +0.2225, -0.1320, -0.3550, -0.3005, +0.0118, +0.2782, +0.4691, -1.3792, +0.1971, -0.0598],
[ +0.0215, +0.1885, -0.5360, -0.1283, +0.4689, +0.1426, -0.2809, -0.8197, +0.1951, -0.1620, +0.0627, +0.2864, -0.3069, -0.1170, +0.0545, -0.4527, -0.6646, -0.1546, -0.1794, -0.5350, -0.1060, -0.0198, -0.5782, -0.2201, +0.0361, -0.2497, -0.1527, -0.1489, +0.1034, +0.0925, +0.0368, -0.0352],
[ +0.2459, +0.3230, -0.0494, -0.5631, +0.0600, -0.3036, -0.5443, +0.1081, -0.2231, +0.0734, +0.0289, +0.4205, -0.6415, -0.1305, -0.0717, +0.2971, +0.0476, -1.3001, +0.5122, -0.0005, -0.3572, +0.0727, +0.1713, -0.4751, -0.3614, -0.0957, -0.0942, +0.0580, +0.2393, +0.0038, +0.1938, -0.1704],
[ +0.3352, -0.0882, -0.0349, -0.6093, +0.4262, -0.1350, -0.0687, -0.2459, -0.5564, -0.2956, +0.1619, -0.0813, -0.5128, -0.2209, +0.3870, -0.0804, +0.7676, -0.1745, -0.3860, -0.5517, -0.6899, -0.6400, +0.6095, -0.5337, +0.3452, -0.6608, +0.0662, +0.1741, +0.1653, -0.4191, +0.1051, -0.3116],
[ -0.0527, -1.3119, +0.3441, -0.0041, -0.5938, -0.4224, +0.3973, +0.4673, -0.0613, -0.0191, +0.1297, -0.2211, -0.0880, +0.0319, +0.0661, -0.2075, +0.4380, +0.3197, +0.0989, +0.2346, -0.0142, -1.2137, +0.1618, -0.3300, +0.4591, +0.4910, +0.3537, -0.5902, -0.0616, +0.2882, -0.0900, -0.0208],
[ -0.7068, -0.7952, +0.4496, +0.1237, -0.2000, -0.5966, +0.3920, +0.3458, +0.0036, -0.0666, -0.3061, -0.1172, +0.0446, +0.1768, -0.5318, +0.2083, +0.3371, +0.1497, +0.4244, +0.3980, +0.2023, -0.8931, +0.1860, -0.6889, -0.3250, +0.1250, +0.1510, -0.3405, -0.4040, +0.1598, -0.9933, +0.0233],
[ -1.2305, -0.3178, +0.0536, -0.0585, -0.7097, +0.3196, +0.2899, +0.8200, +0.0384, +0.1733, -1.1839, -2.2545, +0.0653, +0.1376, -0.1359, -0.1202, -0.0831, -0.5397, +0.1100, +0.1386, -0.1271, -0.6298, +0.1038, -0.1213, -0.1461, -0.4508, +0.5106, -0.8266, -0.6204, +0.3753, -0.4897, -0.0751],
[ -0.3676, -0.5547, +0.0897, -0.0230, -0.3045, -0.1885, -0.5990, +0.3622, -0.2240, -0.1205, -0.3056, +0.7085, +0.0053, -0.1213, -0.3023, +0.1433, -0.2090, -0.0412, +0.2561, +0.1313, -0.2803, +0.2543, +0.0571, -0.9791, -0.0167, -0.2928, -0.3020, -0.2271, +0.0507, -0.1310, -0.6347, -0.0889],
[ -0.2794, +0.0675, -1.0020, -0.2234, +0.3937, -0.2857, +0.1058, -1.0755, -0.0377, -0.2753, -0.0501, -0.0493, -0.2987, -0.2214, +0.2869, -1.0882, -1.2635, -1.2235, -0.5762, -0.4528, -0.1372, -0.0192, -1.3768, +0.2337, +0.2008, -0.2517, -0.3918, -0.6362, -0.1762, -0.9261, +0.1711, -0.0094],
[ -0.1099, -0.2142, -0.0006, -0.4617, -0.0286, +0.3482, -0.7728, -0.4384, +0.0050, -0.0151, +0.1974, +0.2815, -0.5295, -0.2581, +0.3404, -1.6254, -1.3208, -0.1648, -0.5207, +0.4104, -0.2795, +0.0613, -1.5642, -0.1178, -0.1354, +0.0375, +0.3323, +0.0540, +0.2038, -0.3223, +0.4603, -0.3780],
[ -0.3999, -0.3719, +0.1918, -0.4738, -0.0009, +0.0419, +0.1046, +0.2675, +0.1359, -0.2536, -0.3485, -0.3118, -0.3613, +0.0914, -0.4486, +0.2719, +0.2876, -0.0685, +0.4309, +0.1856, +0.4678, -0.3314, +0.0211, +0.2575, +0.5077, -0.1494, +0.5110, -0.6869, -1.4053, +0.3093, -0.2914, -0.1501],
[ +0.3543, +0.3915, +0.0536, +0.3995, +0.2165, -0.1133, -0.1209, +0.0824, -0.0723, -0.0774, -0.4248, -0.0243, -0.1089, -0.1408, +0.2072, -0.1309, -1.5186, -0.4079, -0.0530, -0.3525, +0.6782, +0.1991, -0.0292, +0.1339, -0.1074, +0.2312, +0.1969, +0.4662, +0.5312, -0.3306, +0.0622, +0.1057],
[ -1.1778, +0.2978, +0.0443, +0.1657, +0.1317, -0.1250, -0.0459, +0.0777, +0.1359, -0.0055, +0.2364, -2.3659, +0.2214, -0.1489, -0.3051, -0.5094, +0.1495, +0.3328, +0.1264, -0.0217, +0.2321, -0.6466, -0.1813, +0.5276, +0.1975, +0.3752, +0.1469, -0.8019, +0.2427, +0.1543, +0.2140, -0.1592],
[ -0.7753, -1.3502, +0.3157, +0.1847, +0.0661, -0.5501, +0.3482, +0.6112, +0.0207, +0.0534, -0.2106, -1.0144, -0.0836, -0.0275, -1.0761, +0.2131, +0.3135, +0.3134, +0.1974, +0.0182, +0.1975, -1.1221, +0.2958, -0.2610, +0.0865, +0.3592, +0.4317, -0.3505, -0.4557, +0.3033, -0.5797, -0.2988],
[ +0.4103, -0.0643, +0.0803, +0.2177, +0.1028, -0.2668, +0.0084, -0.2340, -0.2571, +0.0334, +0.3451, -0.0055, +0.0216, -0.1460, +0.5293, -0.2615, -0.3035, +0.1736, -0.4206, -0.2186, +0.1343, +0.6001, -0.0499, -0.2777, -0.0160, -0.4303, -0.2795, +0.1932, +0.4219, -0.0800, +0.1819, -0.1007],
[ -0.7074, -0.0546, +0.4495, +0.1427, +0.3306, +0.0811, -0.5433, -0.0609, -0.2128, -0.1059, -1.0477, -0.4679, -0.1780, -0.1373, -0.3672, +0.0724, -0.0554, -0.5400, +0.0457, -0.0469, -0.0367, -0.4609, +0.1668, -0.0266, -0.9007, +0.2975, +0.5204, -0.0453, -0.1314, -0.0980, +0.1424, -0.1877],
[ +0.0657, +0.1230, -0.2558, +0.3103, -0.0795, -0.1243, +0.1956, +0.0262, -0.2626, -0.0554, +0.3760, +0.3076, -0.4633, +0.0790, +0.2363, -0.3311, +0.1235, -0.1727, -0.2468, +0.0188, -0.1121, -0.2807, -0.5865, -0.4197, +0.1949, -0.4970, -1.0413, -0.1698, +0.1798, +0.2004, -0.0514, +0.0254],
[ -0.1566, -1.1156, +0.4431, -0.1503, -0.5682, +0.1822, -0.1201, +0.5151, -0.1386, -0.1764, +0.2063, -0.8582, +0.3750, -0.1405, +0.0852, +0.2641, -0.1951, -0.0575, -0.4181, +0.2273, +0.1332, -0.2797, +0.5406, -0.0869, +0.2453, +0.0648, +0.2252, -0.0628, -0.6882, -0.0514, -0.4663, -0.0954],
[ -0.4780, +0.5844, +0.1782, -0.0831, +0.1547, -0.0595, -0.5646, -0.0488, -0.1774, -0.0098, +0.1833, +0.3520, -0.3359, -0.1492, +0.1139, -0.1223, -0.5312, -0.5361, +0.1689, -0.2020, +0.1069, +0.2327, +0.2887, +0.0526, -0.5916, -0.2435, -0.2342, +0.3422, +0.4399, -1.1880, +0.1293, -0.1021],
[ -1.2784, -1.8266, +0.0630, -0.3332, -0.5833, -0.3733, +0.3265, +0.1977, +0.0716, -0.2575, +0.0403, -0.1961, +0.1541, -0.2311, -0.1734, -0.1785, +0.0168, +0.1134, +0.0407, -0.1661, +0.5985, -1.9681, +0.1342, +0.3432, +0.3934, +0.0663, +0.3141, -2.0177, -1.7071, +0.2942, -1.0684, -0.0737],
[ +0.1763, +0.2191, +0.2609, +0.0723, +0.1038, -0.2516, -0.9086, +0.1536, +0.0153, +0.1061, +0.1675, +0.3839, -0.5326, +0.2007, -0.4943, -0.1048, +0.1614, -0.4703, +0.3453, -0.7441, -0.6187, +0.4247, +0.1721, -0.1776, -0.0919, -0.8387, +0.0798, -0.0598, +0.2711, -0.0508, +0.1761, +0.0029],
[ -0.2003, +0.2194, -0.6280, +0.1593, +0.1648, -0.1007, +0.3162, -0.3881, -0.1584, -0.0148, +0.7057, +0.0085, +0.3488, +0.0977, +0.4018, -0.8195, -0.1944, +0.4359, -0.6605, -0.1929, +0.2237, +0.1087, -0.4213, -0.7149, +0.3972, -0.1313, -0.2815, -0.7234, -0.0561, -0.5364, +0.0178, +0.0349],
[ +0.0567, +0.1687, +0.0007, +0.2939, -1.3854, +0.0168, +0.1909, +0.4919, -0.4547, +0.0562, -0.1188, +0.1653, -0.0265, -0.0541, -0.1117, -0.3240, +0.2545, +0.6516, +0.0124, -0.1258, -0.0656, -0.3524, +0.0174, +0.3926, +0.1125, +0.2834, -0.1961, -0.3603, +0.1783, -0.0224, -0.6900, -0.1688],
[ +0.0672, +0.6339, -0.3839, +0.0077, +0.8224, -0.3197, -0.0589, -0.1318, +0.0222, -0.1530, +0.1237, +0.4014, -0.1952, -0.1130, +0.4214, -0.2741, +0.2291, +0.0757, +0.0563, -0.0967, +0.4210, +0.5133, +0.0412, -0.9212, +0.1377, -0.4068, -0.3652, +0.4283, +0.6182, -0.6187, +0.1997, +0.1240],
[ -0.0067, +0.3307, -0.7751, -0.2084, +0.4740, -0.0264, -0.0768, -0.9519, -0.0632, -0.0753, +0.3293, +0.5260, -0.6023, +0.0060, +0.2799, -0.2904, -0.8262, -0.6644, -0.3900, -0.1461, +0.4965, +0.3996, -0.7569, +0.0612, +0.5168, -0.5160, -0.4875, +0.3759, +0.0295, +0.1027, +0.6096, -0.0115],
[ -0.0110, +0.4652, -0.1486, -0.6029, +0.2581, -0.3184, -0.3759, +0.3213, -0.2748, -0.0630, +0.0953, +0.2101, -1.2738, -0.1353, +0.2710, -0.2276, +0.2586, -0.2347, -0.3320, +0.0487, -0.2318, -0.1002, +0.1236, +0.2660, -0.1172, +0.1437, -0.0850, +0.1659, -0.2152, -0.0764, +0.2838, -0.1325],
[ +0.0152, -0.0906, -0.1897, -0.3521, -0.1836, -0.1694, -0.4150, -0.1695, +0.0509, -0.0716, +0.3118, +0.2422, -0.5058, -0.0637, -0.1038, -0.2828, -0.0528, -0.2051, +0.2062, -0.2105, -0.7317, +0.1881, -0.2992, -0.0883, +0.0115, -1.5295, -0.1671, +0.0411, +0.0648, -0.0119, -0.2941, +0.0273],
[ +0.5028, +0.1780, -0.4643, -0.0373, +0.3067, -0.1974, +0.2643, -0.2365, -0.2083, +0.0472, +0.4830, +0.0630, +0.2155, -0.0916, +0.6290, -0.4427, -0.6266, +0.3576, -0.3541, -0.2034, +0.3733, +0.8247, -0.5837, -0.4372, +0.2696, -0.4042, -0.3436, +0.0355, -0.2288, -0.6382, +0.7358, -0.1229]
])

weights_dense2_b = np.array([ -0.0730, +0.0456, +0.0877, -0.2607, +0.0029, -0.2705, -0.1420, +0.2403, -0.2135, -0.0646, +0.1378, +0.1105, -0.4639, -0.0583, -0.0872, -0.1473, +0.1460, -0.0234, +0.0740, -0.0745, -0.1283, +0.0316, +0.0361, -0.0726, -0.0304, +0.0417, -0.0313, +0.0935, +0.0815, +0.0814, +0.0818, -0.1111])

weights_final_w = np.array([
[ +1.0397],
[ +0.7049],
[ -0.2128],
[ +0.2172],
[ +0.3027],
[ -0.1991],
[ +0.3398],
[ -0.5932],
[ -0.1439],
[ -0.0236],
[ +0.5679],
[ +0.8571],
[ +0.1934],
[ -0.1652],
[ +0.6933],
[ -0.5510],
[ -1.0587],
[ +0.6996],
[ -0.5009],
[ -0.4000],
[ -0.6958],
[ +0.7716],
[ -0.5342],
[ -0.5095],
[ +0.3040],
[ -1.1986],
[ -0.4900],
[ +0.7726],
[ +0.5871],
[ -0.2533],
[ +0.2633],
[ -0.0004]
])

weights_final_b = np.array([ +0.0190])

if __name__=="__main__":
    demo_run()
