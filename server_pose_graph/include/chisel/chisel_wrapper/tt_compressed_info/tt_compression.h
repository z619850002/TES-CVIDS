#pragma once

#include <iterator>
#include <iostream>
#include <vector>
#include "../../open_chisel/Chisel.h"
#include "../../open_chisel/truncation/QuadraticTruncator.h"
#include "../../open_chisel/truncation/InverseTruncator.h"
#include "../../open_chisel/weighting/ConstantWeighter.h"
#include "../../open_chisel/Chunk.h"



#include <Python.h>

#include <numpy/arrayobject.h>

using namespace std;
using namespace chisel;

PyArrayObject * ComposeArray(  
    PyObject *pValue1, 
    PyObject *pValue2, 
    PyObject *pValue3,
    bool bColor = false);


void ConvertArrayToChunks(PyArrayObject * pRetArray, 
    chisel::ChunkManager * pManager);


void ConvertArrayToChunksColor(
    PyArrayObject * pRetArray,  
    PyArrayObject * pRetArrayR,
    PyArrayObject * pRetArrayG,
    PyArrayObject * pRetArrayB,
    chisel::ChunkManager * pManagerRef);

bool InitializePythonEnvironment();

// bool bSuccess = PyArg_ParseTuple(pyValue,"ii",&r1, &r2);//读取python输出变量（即hello_cpython函数return的结果）
bool DecomposeChunks(
    chisel::ChunkManager  & iManagerRef,
    PyArrayObject * &pK1, 
    PyArrayObject * &pK2, 
    PyArrayObject * &pK3);

bool ExtractData(
    chisel::ChunkManager  & iManagerRef,
    float * & pData,
    vector<int> & gDims);



bool ExtractDataColor(
    chisel::ChunkManager  & iManagerRef,
    vector<float *> & gDatas,
    vector<int> & gDims);


bool DecomposeChunksColor(
    chisel::ChunkManager  & iManagerRef,
    vector<PyArrayObject *> &gK1s,   //drgb
    vector<PyArrayObject *> &gK2s, 
    vector<PyArrayObject *> &gK3s);

