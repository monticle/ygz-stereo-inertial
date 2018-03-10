add_library( ygz-backend
        src/BackendSlidingWindowG2O.cpp
        )
		
target_link_libraries(ygz-backend
        ${THIRD_PARTY_LIBS}
        )

cotire(ygz-backend)