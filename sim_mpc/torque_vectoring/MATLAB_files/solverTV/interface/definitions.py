import numpy
import ctypes

name = "solverTV"
requires_callback = True
lib = "lib/libsolverTV.so"
lib_static = "lib/libsolverTV.a"
c_header = "include/solverTV.h"
nstages = 15

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  4,   1),    4),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (145,   1),  145),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (630,   1),  630),
 ("solver_timeout"      , "dense" , "solverTV_float" , ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("num_of_threads"      , "dense" , "solver_int32_unsigned", ctypes.c_uint  , numpy.uint32 , (  1,   1),    1)]

# Output                | Type    | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("zI"                  , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("z"                   , ""               , ctypes.c_double, numpy.float64,     (130,),  130),
 ("zN"                  , ""               , ctypes.c_double, numpy.float64,     (  5,),    5)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
 ('it2opt', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('res_ineq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('rcompnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('dobj', ctypes.c_double),
 ('dgap', ctypes.c_double),
 ('rdgap', ctypes.c_double),
 ('mu', ctypes.c_double),
 ('mu_aff', ctypes.c_double),
 ('sigma', ctypes.c_double),
 ('lsit_aff', ctypes.c_int),
 ('lsit_cc', ctypes.c_int),
 ('step_aff', ctypes.c_double),
 ('step_cc', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('solver_id', ctypes.c_int * 8)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(10, 4, 8, 42, 0, 0, 8, 8), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(10, 4, 10, 42, 0, 0, 10, 10), 
	(5, 3, 2, 42, 0, 0, 2, 2)
]