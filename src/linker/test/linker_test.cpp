#include "linker.hpp"

#include <fstream>
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

void ProduceTestROM() {
    using namespace gbds::linker;
    using namespace gbds::gbops;
    Header header {
        .starting_point = 0x150,
        .short_title = "TEST",
        .cgb_type = Header::CgbType::BACKWARDS_COMPATIBLE,
        .mapper_type = Header::MapperType::ROM,
        .rom_size_exp = 0,
        .num_ram_banks = Header::RamBanks::NONE
    };
    using opptr = std::unique_ptr<Opcode>;
    opptr ops[] {
        std::make_unique<MoveByte>(RegisterByte::A, (unsigned char)0b10010000),
        std::make_unique<MemHigh>(0x40, true),
        std::make_unique<LoadWord>(RegisterWord::HL, 0x8000),
        std::make_unique<BinOpReg>(BinOp::XOR, RegisterByte::A),
        std::make_unique<StoreByte>(RegisterPointer::HL_INC),
        std::make_unique<MoveByte>(RegisterByte::A, (unsigned char)0x90),
        std::make_unique<BinOpReg>(BinOp::CP, RegisterByte::H),
        std::make_unique<JumpImmediate>(Condition::Z, 0x0000),
        std::make_unique<JumpImmediate>(Condition::ALWAYS, 0x0000)
    };
    OpcodeFunction of {
        "main",
        {std::make_move_iterator(std::begin(ops)), std::make_move_iterator(std::end(ops))},
        {
            {0, ".start"},
            {3, ".inc"}
        },
        {
            {7, {".start", 0}},
            {8, {".inc", 0}}
        }
    };
    Linker linker;
    LinkedFunction lf = linker.LinkFunction(of);
    std::stringstream sstr;
    linker.Write(
        header,
        {
            {"main", 0x150}
        },
        {
            {"main", lf}
        },
        sstr
    );

    std::ofstream outfile("out.gb", std::ios_base::binary);
    sstr.seekg(0);
    std::string str = sstr.str();
    outfile.write(str.c_str(), str.size());
    outfile.close();
}

int main() {
    LinkSimpleFunction();
    ProduceTestROM();
}
