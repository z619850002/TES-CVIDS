import tntorch as tn
import numpy as np
import torch
import time

def tt_decompose_color(data):
	data = torch.Tensor(data)
	t = tn.Tensor(data, ranks_tt=40)  # Random 4D TT tensor of shape 32 x 32 x 32 x 32 and TT-rank 5

	cores = t.cores
	k1 = cores[0].squeeze(0).numpy()
	k2 = cores[1].numpy()
	k3 = cores[2].squeeze(2).numpy()
	return k1, k2, k3

def tt_decompose(data):
	data[data>1] = 1
	data[data<-1] = -1

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
	res[res>1] = 99999.0
	res[res<-1] = 99999.0
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


