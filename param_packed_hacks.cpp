



//static constexpr CHAR_SET
//

namespace mavlink {

class enum param_charset // TODO: map back to ASCII
{
	'A'
	'B'
	'C'
	'D'
	'E'
	'F'
	'G'
	'H'
	'I'
	'J'
	'K'
	'L'
	'M'
	'N'
	'O'
	'P'
	'Q'
	'R'
	'S'
	'T'
	'U'
	'V'
	'W'
	'X'
	'Y'
	'Z'
	'0'
	'1'
	'2'
	'3'
	'4'
	'5'
	'6'
	'7'
	'8'
	'9'
	'_'

	'/' // group delimiter?
}


// param string type
//  internally contains a buffer
//   each character is 6 bits
//   helper to get each character
//   helper to decode the string entirely to ascii


class enum param_type : uint8_t
{
	'bool',
	'int8',
	'uint8',
	'int16',
	'uint16',
	'int32',
	'uint32',
	'int64',
	'string'

	// date?
	// should types be more interesting?
	//  angle in degrees
	//  angle in radians
	//   time in seconds	
}

// size in bits
class enum param_type_size
{
	'bool' = 1,
	'int8' = 8,
	'uint8' = 8,
	'int16' = 16,
	'uint16' = 16,
	'int32' = 32,
	'uint32' = 32,
	'int64' = 64,
	'uint64' = 64,
	'float32' = 32,
	'float64' = 64,
	'string' = 256, // maximum size
}


struct S {
	uint16_t number;		// parameter number during sync
	uint8_t name_length : 5;	// 5 bit name length
	uint8_t type : 3;		// 3 bit param type

	uint8_t buffer_start[1]; // marker for start of buffer
};

struct param_header // size 3 bytes + up to 247 bytes of param name + param value
{
	uint16_t number;
	uint8_t name_length;
	param_type type;

	uint8_t buffer_start[1]; // marker for start of buffer

	// absolute min size is 1 byte
	//  1 character param name (6 bits) + 1 bit boolean value
};

struct param_group
{
	uint16 number; // number of this parameter group
	uint16 parent_group;  // parant group (if any), 0 if no parent
	char name[16];
	
	uint16 number_params; // number of parameters that belong to this group
};


struct mavlink_parameter_packed_sync
{

	uint8_t number;
	uint8_t total_size;
	uint16_t parameter_group_num; // group these parameters belong to

	uint8_t buffer[250];

}

// name is a variable size param_charset array, eg param_charset[16]
// value is a variable size array
//     




// PACK

//  iterate through parameters to fill array 
//   mav param -> packed param


} // namespace mavlink