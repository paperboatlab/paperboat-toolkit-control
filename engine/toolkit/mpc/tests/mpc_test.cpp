#include <filesystem>
#include <fstream>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "paperboat/toolkit/block.hpp"
#include "paperboat/toolkit/mpc.hpp"

namespace tk = paperboat::toolkit;

TEST_CASE("MPC solves a one-state QP every step") {
  const auto path = std::filesystem::temp_directory_path() / "paperboat_control_mpc.json";
  {
    std::ofstream out(path);
    out << R"({"A": [[1]], "B": [[1]], "Q": [[1]], "R": [[0.1]], "N": 5,
                "u_min": [-1], "u_max": [1]})";
  }

  tk::Registry registry = tk::builtins();
  tk::mpc::register_mpc(registry);
  const tk::Template* t = registry.find("mpc/mpc");
  REQUIRE(t != nullptr);
  REQUIRE(t->make);
  auto block = t->make({}, tk::Config{{"file", path.string()}});

  const tk::Values out = block->step(tk::Context{}, {{"y", tk::Vector{1.0}}, {"ref", tk::Vector{0.0}}});
  const tk::Vector* u = tk::get_vector(out, "u");
  REQUIRE(u != nullptr);
  REQUIRE(u->size() == 1);
  CHECK((*u)[0] < 0.0);
  CHECK(tk::get_text(out, "status").find("solved") != std::string::npos);
  std::filesystem::remove(path);
}
