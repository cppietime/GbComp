#include "linker.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

void ExpectLinkedFunction(gbds::linker::LinkedFunction actual, gbds::linker::LinkedFunction expected) {
    if (actual != expected) {
        throw "Linked functions do not match";
    }
}

void LinkSimpleFunction() {
    using namespace gbds;
    auto op1 = std::make_unique<gbops::ByteOpcode>(gbops::ByteOp::NOP);
    using opptr = std::unique_ptr<gbops::Opcode>;
    opptr ops[] {
        std::make_unique<gbops::ByteOpcode>(gbops::ByteOp::NOP),
        std::make_unique<gbops::LoadWord>(gbops::RegisterWord::HL, 0),
        std::make_unique<gbops::JumpImmediate>(gbops::Condition::C, 0),
        std::make_unique<gbops::Call>(gbops::Condition::ALWAYS, 0)
    };
    std::vector<std::unique_ptr<gbops::Opcode>> vec {std::make_move_iterator(std::begin(ops)), std::make_move_iterator(std::end(ops))};
    linker::OpcodeFunction opcode_func {
        "function",
        std::move(vec),
        {
            {2, ".jump"}
        },
        {
            {1, {"glob", 0}},
            {2, {".jump", 0}},
            {3, {"glob", 5}}
        }
    };
    linker::Linker linker;
    linker::LinkedFunction linked_func = linker.LinkFunction(opcode_func);

    uint8_t expected_binary[] {0x00, 0x21, 0x00, 0x00, 0xDA, 0x00, 0x00, 0xCD, 0x00, 0x00};
    std::map<size_t, linker::Patch> expected_patches {
        {2, {"glob", 0}},
        {5, {"function", 4}},
        {8, {"glob", 5}}
    };
    linker::LinkedFunction expected_func {{std::begin(expected_binary), std::end(expected_binary)}, expected_patches};
}

int main() {
    LinkSimpleFunction();
}
