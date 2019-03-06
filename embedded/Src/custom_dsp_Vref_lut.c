#include "custom_dsp.h"

// 125 kHz reference signals, Vref0: cos, Vref90: sin
float64_t VrefI_fexc125k_fs20M[160] =
{
		1.0000000000000000,0.9992290362407229,0.9969173337331280,0.9930684569549263,0.9876883405951378,0.9807852804032304,0.9723699203976766,0.9624552364536473,0.9510565162951535,0.9381913359224842,0.9238795325112867,0.9081431738250813,0.8910065241883679,0.8724960070727972,0.8526401643540923,0.8314696123025452,0.8090169943749475,0.7853169308807449,0.7604059656000309,0.7343225094356856,0.7071067811865475,0.6788007455329417,0.6494480483301837,0.6190939493098340,0.5877852522924731,0.5555702330196022,0.5224985647159488,0.4886212414969550,0.4539904997395467,0.4186597375374281,0.3826834323650898,0.3461170570774930,0.3090169943749474,0.2714404498650743,0.2334453638559054,0.1950903220161282,0.1564344650402309,0.1175373974578376,0.0784590957278449,0.0392598157590686,0.0000000000000000,-0.0392598157590686,-0.0784590957278449,-0.1175373974578376,-0.1564344650402309,-0.1950903220161282,-0.2334453638559054,-0.2714404498650743,-0.3090169943749474,-0.3461170570774930,-0.3826834323650898,-0.4186597375374281,-0.4539904997395467,-0.4886212414969550,-0.5224985647159488,-0.5555702330196022,-0.5877852522924731,-0.6190939493098340,-0.6494480483301837,-0.6788007455329417,-0.7071067811865475,-0.7343225094356856,-0.7604059656000309,-0.7853169308807449,-0.8090169943749475,-0.8314696123025452,-0.8526401643540923,-0.8724960070727972,-0.8910065241883679,-0.9081431738250813,-0.9238795325112867,-0.9381913359224842,-0.9510565162951535,-0.9624552364536473,-0.9723699203976766,-0.9807852804032304,-0.9876883405951378,-0.9930684569549263,-0.9969173337331280,-0.9992290362407229,-1.0000000000000000,-0.9992290362407229,-0.9969173337331280,-0.9930684569549263,-0.9876883405951378,-0.9807852804032304,-0.9723699203976766,-0.9624552364536473,-0.9510565162951535,-0.9381913359224842,-0.9238795325112867,-0.9081431738250813,-0.8910065241883679,-0.8724960070727972,-0.8526401643540923,-0.8314696123025452,-0.8090169943749475,-0.7853169308807449,-0.7604059656000309,-0.7343225094356856,-0.7071067811865475,-0.6788007455329417,-0.6494480483301837,-0.6190939493098340,-0.5877852522924731,-0.5555702330196022,-0.5224985647159488,-0.4886212414969550,-0.4539904997395467,-0.4186597375374281,-0.3826834323650898,-0.3461170570774930,-0.3090169943749474,-0.2714404498650743,-0.2334453638559054,-0.1950903220161282,-0.1564344650402309,-0.1175373974578376,-0.0784590957278449,-0.0392598157590686,0.0000000000000000,0.0392598157590686,0.0784590957278449,0.1175373974578376,0.1564344650402309,0.1950903220161282,0.2334453638559054,0.2714404498650743,0.3090169943749474,0.3461170570774930,0.3826834323650898,0.4186597375374281,0.4539904997395467,0.4886212414969550,0.5224985647159488,0.5555702330196022,0.5877852522924731,0.6190939493098340,0.6494480483301837,0.6788007455329417,0.7071067811865475,0.7343225094356856,0.7604059656000309,0.7853169308807449,0.8090169943749475,0.8314696123025452,0.8526401643540923,0.8724960070727972,0.8910065241883679,0.9081431738250813,0.9238795325112867,0.9381913359224842,0.9510565162951535,0.9624552364536473,0.9723699203976766,0.9807852804032304,0.9876883405951378,0.9930684569549263,0.9969173337331280,0.9992290362407229
};
float64_t VrefQ_fexc125k_fs20M[160] =
{
		0.0000000000000000,0.0392598157590686,0.0784590957278449,0.1175373974578376,0.1564344650402309,0.1950903220161282,0.2334453638559054,0.2714404498650743,0.3090169943749474,0.3461170570774930,0.3826834323650898,0.4186597375374281,0.4539904997395467,0.4886212414969550,0.5224985647159488,0.5555702330196022,0.5877852522924731,0.6190939493098340,0.6494480483301837,0.6788007455329417,0.7071067811865475,0.7343225094356856,0.7604059656000309,0.7853169308807449,0.8090169943749475,0.8314696123025452,0.8526401643540923,0.8724960070727972,0.8910065241883679,0.9081431738250813,0.9238795325112867,0.9381913359224842,0.9510565162951535,0.9624552364536473,0.9723699203976766,0.9807852804032304,0.9876883405951378,0.9930684569549263,0.9969173337331280,0.9992290362407229,1.0000000000000000,0.9992290362407229,0.9969173337331280,0.9930684569549263,0.9876883405951378,0.9807852804032304,0.9723699203976766,0.9624552364536473,0.9510565162951535,0.9381913359224842,0.9238795325112867,0.9081431738250813,0.8910065241883679,0.8724960070727972,0.8526401643540923,0.8314696123025452,0.8090169943749475,0.7853169308807449,0.7604059656000309,0.7343225094356856,0.7071067811865475,0.6788007455329417,0.6494480483301837,0.6190939493098340,0.5877852522924731,0.5555702330196022,0.5224985647159488,0.4886212414969550,0.4539904997395467,0.4186597375374281,0.3826834323650898,0.3461170570774930,0.3090169943749474,0.2714404498650743,0.2334453638559054,0.1950903220161282,0.1564344650402309,0.1175373974578376,0.0784590957278449,0.0392598157590686,0.0000000000000000,-0.0392598157590686,-0.0784590957278449,-0.1175373974578376,-0.1564344650402309,-0.1950903220161282,-0.2334453638559054,-0.2714404498650743,-0.3090169943749474,-0.3461170570774930,-0.3826834323650898,-0.4186597375374281,-0.4539904997395467,-0.4886212414969550,-0.5224985647159488,-0.5555702330196022,-0.5877852522924731,-0.6190939493098340,-0.6494480483301837,-0.6788007455329417,-0.7071067811865475,-0.7343225094356856,-0.7604059656000309,-0.7853169308807449,-0.8090169943749475,-0.8314696123025452,-0.8526401643540923,-0.8724960070727972,-0.8910065241883679,-0.9081431738250813,-0.9238795325112867,-0.9381913359224842,-0.9510565162951535,-0.9624552364536473,-0.9723699203976766,-0.9807852804032304,-0.9876883405951378,-0.9930684569549263,-0.9969173337331280,-0.9992290362407229,-1.0000000000000000,-0.9992290362407229,-0.9969173337331280,-0.9930684569549263,-0.9876883405951378,-0.9807852804032304,-0.9723699203976766,-0.9624552364536473,-0.9510565162951535,-0.9381913359224842,-0.9238795325112867,-0.9081431738250813,-0.8910065241883679,-0.8724960070727972,-0.8526401643540923,-0.8314696123025452,-0.8090169943749475,-0.7853169308807449,-0.7604059656000309,-0.7343225094356856,-0.7071067811865475,-0.6788007455329417,-0.6494480483301837,-0.6190939493098340,-0.5877852522924731,-0.5555702330196022,-0.5224985647159488,-0.4886212414969550,-0.4539904997395467,-0.4186597375374281,-0.3826834323650898,-0.3461170570774930,-0.3090169943749474,-0.2714404498650743,-0.2334453638559054,-0.1950903220161282,-0.1564344650402309,-0.1175373974578376,-0.0784590957278449,-0.0392598157590686
};

// 250 kHz reference signals, Vref0: cos, Vref90: sin
float64_t VrefI_fexc250k_fs20M[80] =
{
		1.0000000000000000,0.9969173337331280,0.9876883405951378,0.9723699203976766,0.9510565162951535,0.9238795325112867,0.8910065241883679,0.8526401643540923,0.8090169943749475,0.7604059656000309,0.7071067811865475,0.6494480483301837,0.5877852522924731,0.5224985647159488,0.4539904997395467,0.3826834323650898,0.3090169943749474,0.2334453638559054,0.1564344650402309,0.0784590957278449,0.0000000000000000,-0.0784590957278449,-0.1564344650402309,-0.2334453638559054,-0.3090169943749474,-0.3826834323650898,-0.4539904997395467,-0.5224985647159488,-0.5877852522924731,-0.6494480483301837,-0.7071067811865475,-0.7604059656000309,-0.8090169943749475,-0.8526401643540923,-0.8910065241883679,-0.9238795325112867,-0.9510565162951535,-0.9723699203976766,-0.9876883405951378,-0.9969173337331280,-1.0000000000000000,-0.9969173337331280,-0.9876883405951378,-0.9723699203976766,-0.9510565162951535,-0.9238795325112867,-0.8910065241883679,-0.8526401643540923,-0.8090169943749475,-0.7604059656000309,-0.7071067811865475,-0.6494480483301837,-0.5877852522924731,-0.5224985647159488,-0.4539904997395467,-0.3826834323650898,-0.3090169943749474,-0.2334453638559054,-0.1564344650402309,-0.0784590957278449,0.0000000000000000,0.0784590957278449,0.1564344650402309,0.2334453638559054,0.3090169943749474,0.3826834323650898,0.4539904997395467,0.5224985647159488,0.5877852522924731,0.6494480483301837,0.7071067811865475,0.7604059656000309,0.8090169943749475,0.8526401643540923,0.8910065241883679,0.9238795325112867,0.9510565162951535,0.9723699203976766,0.9876883405951378,0.9969173337331280
};
float64_t VrefQ_fexc250k_fs20M[80] =
{
		0.0000000000000000,0.0784590957278449,0.1564344650402309,0.2334453638559054,0.3090169943749474,0.3826834323650898,0.4539904997395467,0.5224985647159488,0.5877852522924731,0.6494480483301837,0.7071067811865475,0.7604059656000309,0.8090169943749475,0.8526401643540923,0.8910065241883679,0.9238795325112867,0.9510565162951535,0.9723699203976766,0.9876883405951378,0.9969173337331280,1.0000000000000000,0.9969173337331280,0.9876883405951378,0.9723699203976766,0.9510565162951535,0.9238795325112867,0.8910065241883679,0.8526401643540923,0.8090169943749475,0.7604059656000309,0.7071067811865475,0.6494480483301837,0.5877852522924731,0.5224985647159488,0.4539904997395467,0.3826834323650898,0.3090169943749474,0.2334453638559054,0.1564344650402309,0.0784590957278449,0.0000000000000000,-0.0784590957278449,-0.1564344650402309,-0.2334453638559054,-0.3090169943749474,-0.3826834323650898,-0.4539904997395467,-0.5224985647159488,-0.5877852522924731,-0.6494480483301837,-0.7071067811865475,-0.7604059656000309,-0.8090169943749475,-0.8526401643540923,-0.8910065241883679,-0.9238795325112867,-0.9510565162951535,-0.9723699203976766,-0.9876883405951378,-0.9969173337331280,-1.0000000000000000,-0.9969173337331280,-0.9876883405951378,-0.9723699203976766,-0.9510565162951535,-0.9238795325112867,-0.8910065241883679,-0.8526401643540923,-0.8090169943749475,-0.7604059656000309,-0.7071067811865475,-0.6494480483301837,-0.5877852522924731,-0.5224985647159488,-0.4539904997395467,-0.3826834323650898,-0.3090169943749474,-0.2334453638559054,-0.1564344650402309,-0.0784590957278449
};

// 500 kHz reference signals, Vref0: cos, Vref90: sin
float64_t VrefI_fexc500k_fs20M[40] =
{
		1.0000000000000000, 0.9876883405951378, 0.9510565162951535, 0.8910065241883679, 0.8090169943749475,
		0.7071067811865475, 0.5877852522924731, 0.4539904997395467, 0.3090169943749474, 0.1564344650402309, 0.0000000000000000, -0.1564344650402309,
		-0.3090169943749474, -0.4539904997395467, -0.5877852522924731, -0.7071067811865475, -0.8090169943749475, -0.8910065241883679, -0.9510565162951535,
		-0.9876883405951378, -1.0000000000000000, -0.9876883405951378, -0.9510565162951535, -0.8910065241883679, -0.8090169943749475, -0.7071067811865475,
		-0.5877852522924731, -0.4539904997395467, -0.3090169943749474, -0.1564344650402309, 0.0000000000000000, 0.1564344650402309, 0.3090169943749474,
		0.4539904997395467, 0.5877852522924731, 0.7071067811865475, 0.8090169943749475, 0.8910065241883679, 0.9510565162951535, 0.9876883405951378
};
float64_t VrefQ_fexc500k_fs20M[40] =
{
		0.0000000000000000, 0.1564344650402309, 0.3090169943749474, 0.4539904997395467, 0.5877852522924731,
		0.7071067811865475, 0.8090169943749475, 0.8910065241883679, 0.9510565162951535, 0.9876883405951378, 1.0000000000000000, 0.9876883405951378,
		0.9510565162951535, 0.8910065241883679, 0.8090169943749475, 0.7071067811865475, 0.5877852522924731, 0.4539904997395467, 0.3090169943749474,
		0.1564344650402309, 0.0000000000000000, -0.1564344650402309, -0.3090169943749474, -0.4539904997395467, -0.5877852522924731, -0.7071067811865475,
		-0.8090169943749475, -0.8910065241883679, -0.9510565162951535, -0.9876883405951378, -1.0000000000000000, -0.9876883405951378, -0.9510565162951535,
		-0.8910065241883679, -0.8090169943749475, -0.7071067811865475, -0.5877852522924731, -0.4539904997395467, -0.3090169943749474, -0.1564344650402309
};

// 1 MHz reference signals, Vref0: cos, Vref90: sin
float64_t VrefI_fexc1M_fs20M[20] =
{
		1.0000000000000000, 0.9510565162951535, 0.8090169943749475, 0.5877852522924731, 0.3090169943749474,
		0.0000000000000000, -0.3090169943749474, -0.5877852522924731, -0.8090169943749475, -0.9510565162951535, -1.0000000000000000, -0.9510565162951535,
		-0.8090169943749475, -0.5877852522924731, -0.3090169943749474, 0.0000000000000000, 0.3090169943749474, 0.5877852522924731, 0.8090169943749475,
		0.9510565162951535
};
float64_t VrefQ_fexc1M_fs20M[20] =
{
		0.0000000000000000, 0.3090169943749474, 0.5877852522924731, 0.8090169943749475, 0.9510565162951535,
		1.0000000000000000, 0.9510565162951535, 0.8090169943749475, 0.5877852522924731, 0.3090169943749474, 0.0000000000000000, -0.3090169943749474,
		-0.5877852522924731, -0.8090169943749475, -0.9510565162951535, -1.0000000000000000, -0.9510565162951535, -0.8090169943749475, -0.5877852522924731,
		-0.3090169943749474
};

// 2 MHz reference signals, Vref0: cos, Vref90: sin
float64_t VrefI_fexc2M_fs20M[10] =
 {
		 1.0000000000000000, 0.8090169943749475, 0.3090169943749474, -0.3090169943749474, -0.8090169943749475,
		-1.0000000000000000, -0.8090169943749475, -0.3090169943749474, 0.3090169943749474, 0.8090169943749475
 };
float64_t VrefQ_fexc2M_fs20M[10] =
 {
		 0.0000000000000000, 0.5877852522924731, 0.9510565162951535, 0.9510565162951535, 0.5877852522924731,
		0.0000000000000000, -0.5877852522924731, -0.9510565162951535, -0.9510565162951535, -0.5877852522924731
 };

// Look-up table address dictionary
float64_t *Vref_fexc125k_fs20M[2] = {VrefI_fexc125k_fs20M, VrefQ_fexc125k_fs20M};
float64_t *Vref_fexc250k_fs20M[2] = {VrefI_fexc250k_fs20M, VrefQ_fexc250k_fs20M};
float64_t *Vref_fexc500k_fs20M[2] = {VrefI_fexc500k_fs20M, VrefQ_fexc500k_fs20M};
float64_t *Vref_fexc1M_fs20M[2] = {VrefI_fexc1M_fs20M, VrefQ_fexc1M_fs20M};
float64_t *Vref_fexc2M_fs20M[2] = {VrefI_fexc2M_fs20M, VrefQ_fexc2M_fs20M};
