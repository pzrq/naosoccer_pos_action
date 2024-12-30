# Build test_parser
ament_add_gtest(test_parser
  test_parser.cpp)

target_link_libraries(test_parser
  nao_pos_server_node
)
