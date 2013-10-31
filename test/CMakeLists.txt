macro(add_xares_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} xares )
    add_test(${name} ${name})
endmacro()

add_xares_test(test_xares)
