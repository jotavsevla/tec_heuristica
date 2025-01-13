cmake_minimum_required(VERSION 3.10)
project(tec_heuristica_pj1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_ARCHITECTURES "x86_64")

# Adiciona o subdiretório do TinyXML2
add_subdirectory(lab/tinyxml2)

# Adiciona os diretórios de include
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/lab/tinyxml2
)

# Lista todos os arquivos fonte
set(SOURCES
        src/main.cpp
        src/Interface.cpp
        src/Edge.cpp
        src/Route.cpp
        src/PhysarumSolver.cpp
        src/SolutionValidator.cpp
        src/PhysarumSolverV3.cpp
)
# configurando Adress Sanitizer
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

# Cria o executável
add_executable(${PROJECT_NAME} ${SOURCES})

# Link com a biblioteca TinyXML2
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2)

# Configurações específicas para macOS
if(APPLE)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum OS X deployment version")
endif()
#if(APPLE)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
#    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
#endif()





# Configura os diretórios de saída
set_target_properties(${PROJECT_NAME}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Cria os diretórios necessários
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/entradas)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin/resultados)
