#! /usr/bin/env  python3
#coding:utf-8
import rospy
import torch
import numpy as np

# from std_msgs.msg import String
from chisel_msg.msg import Tensor
from chisel_msg.msg import CompressedTensor, TTUncompressedChiselMap, TTChiselMap



import tntorch as tn
import torch
import time

import pickle

def tt_decompose_color(data_2):
    data = data_2.copy()
    data = torch.Tensor(data)
    t = tn.Tensor(data, ranks_tt=40)  # Random 4D TT tensor of shape 32 x 32 x 32 x 32 and TT-rank 5

    cores = t.cores
    k1 = cores[0].squeeze(0).numpy()
    k2 = cores[1].numpy()
    k3 = cores[2].squeeze(2).numpy()
    return k1, k2, k3

def tt_decompose(data_2):
    data = data_2.copy()
    data[data>0.99] = 1
    data[data<-0.99] = -1

    data = torch.Tensor(data)
    t = tn.Tensor(data, ranks_tt=40)  # Random 4D TT tensor of shape 32 x 32 x 32 x 32 and TT-rank 5

    cores = t.cores
    k1 = cores[0].squeeze(0).numpy()
    k2 = cores[1].numpy()
    k3 = cores[2].squeeze(2).numpy()
    return k1, k2, k3

def tt_compose(k1, k2, k3):
    
    k1 = torch.Tensor(k1)
    k2 = torch.Tensor(k2)   
    k3 = torch.Tensor(k3)

    k = torch.einsum('ai,ibj->abj',(k1,k2))
    k = torch.einsum('aib,bj->aij',(k,k3))
    res = k.numpy()
    res[res>0.9] = 99999.0
    res[res<-0.9] = 99999.0
    # print('res shape: ')
    # print(res.shape)
    # np.save('/home/kyrie/Documents/Data/Compress/res_origin.npy', res)
    return res
    
def tt_compose_color(k1, k2, k3):
    k1 = torch.Tensor(k1)
    k2 = torch.Tensor(k2)   
    k3 = torch.Tensor(k3)

    k = torch.einsum('ai,ibj->abj',(k1,k2))
    k = torch.einsum('aib,bj->aij',(k,k3))
    res = k.numpy()
    # print('res color shape: ')
    # print(res.shape)
    return res

def tt_compress(data, index):
    # print('Enter1')

    # np.save('/home/kyrie/Documents/Data/Compress/res' + str(index) + '_origin.npy', data)
    data = torch.Tensor(data)

    if index == 1:
        data[data>1] = 1
        data[data<-1] = -1

    t = tn.Tensor(data, ranks_tt=80)  # Random 4D TT tensor of shape 32 x 32 x 32 x 32 and TT-rank 5
    # print('Enter2')

    cores = t.cores
    k1 = cores[0].squeeze(0)
    k2 = cores[1]
    k3 = cores[2].squeeze(2)
    # print('Enter3')

    k = torch.einsum('ai,ibj->abj',(k1,k2))
    k = torch.einsum('aib,bj->aij',(k,k3))
    res = k.numpy()


    if index == 1:
        res[res>1] = 99999.0
        res[res<-1] = 99999.0

    end = time.time()
    # print('Enter4')
    # np.save('/home/kyrie/Documents/Data/Compress/res' + str(index) + '.npy', res)
    # print('Finish')
    return res





# pub_K = rospy.Publisher('compressed_tensor', CompressedTensor, queue_size=10)
pub_compressed_map = rospy.Publisher('/pose_graph/uncompressed_sub_map', TTUncompressedChiselMap, queue_size=10)


b_first = True

def callback_submap(chisel_map):
    #convert the TTChiselMap to TTUncompressedChiselMap
    #Generate the compressed TTChiselMap

    uc_chiselmap = TTUncompressedChiselMap()
    uc_chiselmap.header = chisel_map.header
    uc_chiselmap.agentNum = chisel_map.agentNum
    uc_chiselmap.publishIndices = chisel_map.publishIndices
    uc_chiselmap.chunkValues = chisel_map.chunkValues
    uc_chiselmap.lengthChunks = chisel_map.lengthChunks
    uc_chiselmap.chunkSize = chisel_map.chunkSize
    uc_chiselmap.voxelResolution = chisel_map.voxelResolution
    uc_chiselmap.useColor = chisel_map.useColor

    uc_chiselmap.refPosition_wc = chisel_map.refPosition_wc
    uc_chiselmap.refOrientation_wc = chisel_map.refOrientation_wc

    #recover the tensors
    compressed_tensor_list = []
    compressed_tensor_list.append(chisel_map.tensorDist)
    compressed_tensor_list.append(chisel_map.tensorR)
    compressed_tensor_list.append(chisel_map.tensorG)
    compressed_tensor_list.append(chisel_map.tensorG)

    composed_tensor_list = []
    for i in range(4):
        ct = compressed_tensor_list[i]
        K1 = np.array(ct.K1.datas) 
        K2 = np.array(ct.K2.datas)
        K3 = np.array(ct.K3.datas)
        shape1 = (ct.K1.shape[0], ct.K1.shape[1])
        shape2 = (ct.K2.shape[0], ct.K2.shape[1], ct.K2.shape[2])
        shape3 = (ct.K3.shape[0], ct.K3.shape[1])
        #reshaped tensor
        k1_t = K1.reshape(shape1[0], shape1[1])
        k2_t = K2.reshape(shape2[0], shape2[1], shape2[2])
        k3_t = K3.reshape(shape3[0], shape3[1])
        #compose the tensors
        if (i==0):
            composed_tensor = tt_compose(k1_t, k2_t, k3_t)

            # np.save('/home/kyrie/Documents/DataSet/Submap/tensors/transform.npy', composed_tensor)
        else:
            composed_tensor = tt_compose_color(k1_t, k2_t, k3_t)

        composed_tensor = composed_tensor.astype(np.float64)

        composed_object = tuple(composed_tensor.reshape(composed_tensor.shape[0] * composed_tensor.shape[1] * composed_tensor.shape[2]).astype(np.int16))

        uncompressed_tensor = Tensor()
        uncompressed_tensor.datas = composed_object
        uncompressed_tensor.shape = (composed_tensor.shape[0], composed_tensor.shape[1], composed_tensor.shape[2])
        uncompressed_tensor.dims = 3
        uncompressed_tensor.submap_index = ct.K1.submap_index
        uncompressed_tensor.color_index = ct.K1.color_index  

        composed_tensor_list.append(uncompressed_tensor)

    uc_chiselmap.tensorDist = composed_tensor_list[0]
    uc_chiselmap.tensorR = composed_tensor_list[1]
    uc_chiselmap.tensorG = composed_tensor_list[2]
    uc_chiselmap.tensorB = composed_tensor_list[3]

    # global b_first
    # if b_first:
    #     b_first = False
    #     a_file = open("/home/kyrie/Documents/DataSet/Submap/tensors/data2.pkl", "wb")
    #     pickle.dump(composed_tensor_list[0].datas, a_file)

    #     pickle.dump(uc_chiselmap.voxelResolution, a_file)
        
    pub_compressed_map.publish(uc_chiselmap)
    for i in range(10):
        print('publish obtained uncompressed map')



def listener():

    rospy.init_node('compress_listener', anonymous=True)

    rospy.Subscriber('/pose_graph/compressed_sub_map', TTChiselMap, callback_submap)

    # chisel_msg::TTUncompressedChiselMap>("compress_map

    rospy.spin()

if __name__ == '__main__':

    # b_first = True
    pub_compressed_map = rospy.Publisher('/pose_graph/uncompressed_sub_map', TTUncompressedChiselMap, queue_size=10)



    listener()

