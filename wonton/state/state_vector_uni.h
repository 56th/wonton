/*
This file is part of the Ristra Wonton project.
Please see the license file at the root of this repository, or at:
    https://github.com/laristra/wonton/blob/master/LICENSE
*/

#ifndef WONTON_STATE_VECTOR_UNI_H_
#define WONTON_STATE_VECTOR_UNI_H_

#include <string>
#include <typeinfo>
#include <vector>

#include "wonton/support/wonton.h"
#include "wonton/state/state_vector_base.h"

namespace Wonton {  

/*!
	This class implements a state vector for a single material field, meaning there
	is only one number per mesh entity. The vector of data needs to be the same size 
	as the number of entitites in the mesh. The field can be define on the cells, 
	nodes, etc..
*/
template <class T=double>
class StateVectorUni : public StateVectorBase {

 public:
  
  	/*!
  		@brief Constructor for StateVectorUni
  		@param[in] kind		the entity kind (CELL, NODE,...) of the data
  		@param[in] data		the vector of data
  	*/
		StateVectorUni(
			std::string name, 
			Entity_kind kind=Entity_kind::CELL,
			std::vector<T> data=std::vector<T>()
		) : StateVectorBase(name, Field_type::MESH_FIELD, kind),data_(data) {}
		

		//! Destructor
		~StateVectorUni() {}
		
		// print
		std::ostream & print(std::ostream & os) const {
		  os << "UniStateVector\n";
		  return os;
		}
		  
		// get the data type
		const std::type_info& data_type() {
			const std::type_info& ti = typeid(T);
			return ti;
		}
		
		/*!
			@brief Return a reference to the data in the state vector.
			@return a reference to the vector of data in the state vector

			Return a reference to the data in the state vector.
		*/
		std::vector<T>& get_data() { return data_; }
 	
 private:
 
 		std::vector<T> data_;
 
};

}

#endif //WONTON_STATE_VECTOR_UNI_H_

