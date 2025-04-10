import numpy
import ctypes

name = "solverMPCC"
requires_callback = True
lib = "lib/libsolverMPCC.so"
lib_static = "lib/libsolverMPCC.a"
c_header = "include/solverMPCC.h"
nstages = 51

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (859,   1),  859),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (21471,   1), 21471),
 ("solver_timeout"      , "dense" , "solverMPCC_float", ctypes.c_double, numpy.float64, (  1,   1),    1),
 ("num_of_threads"      , "dense" , "solver_int32_unsigned", ctypes.c_uint  , numpy.uint32 , (  1,   1),    1)]

# Output                | Type    | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("z"                   , ""               , ctypes.c_double, numpy.float64,     (833,),  833),
 ("zN_1"                , ""               , ctypes.c_double, numpy.float64,     ( 15,),   15),
 ("zN"                  , ""               , ctypes.c_double, numpy.float64,     ( 11,),   11)]

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
	(17, 14, 2, 421, 0, 0, 2, 2), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(17, 14, 3, 421, 5, 2, 2, 3), 
	(15, 10, 1, 421, 5, 2, 0, 1), 
	(11, 0, 1, 421, 2, 0, 0, 1)
]