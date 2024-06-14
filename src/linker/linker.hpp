#ifndef _LINKER_HPP
#define _LINKER_HPP

#include <array>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <span>
#include <string>
#include <variant>
#include <vector>

#define e2i(e) static_cast<int>(e)

namespace gbds
{

    inline void WriteByte(std::ostream &stream, uint8_t byte)
    {
        stream.write(reinterpret_cast<char *>(&byte), 1);
    }

    inline void WriteWordLE(std::ostream &stream, uint16_t value)
    {
        uint8_t bytes[]{value & 0xff, value >> 8};
        stream.write(reinterpret_cast<char *>(bytes), 2);
    }

    namespace gbops
    {

        enum class RegisterByte
        {
            B = 0,
            C = 1,
            D = 2,
            E = 3,
            H = 4,
            L = 5,
            PTR_HL = 6,
            A = 7
        };

        extern std::string register_byte_names[];

        enum class RegisterPointer
        {
            BC,
            DE,
            HL_INC,
            HL_DEC,
            C
        };

        extern std::string register_pointer_names[];

        enum class RegisterWord
        {
            BC,
            DE,
            HL,
            SP_AF
        };

        extern std::string register_word_names[];
        extern std::string push_pop_names[];

        enum class BinOp
        {
            ADD,
            ADC,
            SUB,
            SBC,
            AND,
            XOR,
            OR,
            CP
        };

        extern std::string bin_op_names[];

        enum class RotOp
        {
            RLC,
            RRC,
            RL,
            RR,
            SLA,
            SRA,
            SWAP,
            SRL
        };

        extern std::string rot_op_names[];

        enum class BitOp
        {
            BIT,
            RES,
            SET
        };

        extern std::string bit_op_names[];

        enum class Condition
        {
            ALWAYS,
            NZ,
            Z,
            NC,
            C
        };

        extern std::string condition_names[];

        enum class RstVector
        {
            H00,
            H08,
            H10,
            H18,
            H20,
            H28,
            H30,
            H38
        };

        enum class Bit
        {
            B0,
            B1,
            B2,
            B3,
            B4,
            B5,
            B6,
            B7
        };

        enum class Flag
        {
            ZERO,
            SUBTRACT,
            HALF,
            CARRY,
            NUM_FLAGS
        };

        enum class FlagAction
        {
            IGNORE,
            SET,
            CLEAR,
            UNKNOWN
        };

        union RegisterCobbling
        {
            struct
            {
                uint8_t b : 1;
                uint8_t c : 1;
                uint8_t d : 1;
                uint8_t e : 1;
                uint8_t h : 1;
                uint8_t l : 1;
                uint8_t sp : 1;
                uint8_t a : 1;
            };
            uint8_t byte;
        };

        extern RegisterCobbling cobble_units[];
        extern RegisterCobbling cobble_words[];

        using FlagCobbling = std::array<FlagAction, e2i(Flag::NUM_FLAGS)>;

        enum class ByteOp
        {
            NOP,
            STOP,
            HALT,
            DI,
            EI,
            RLCA,
            RRCA,
            RLA,
            RRA,
            DAA,
            CPL,
            SCF,
            CCF,
            JP_HL,
            MOV_SP_HL
        };

        struct ByteOpT
        {
            const std::string name;
            const uint8_t opcode;
            const RegisterCobbling register_cobbling;
            const FlagCobbling flag_cobbling;
        };

        extern ByteOpT byte_ops[];

        using ImmediateByte = uint8_t;
        using ImmediateWord = uint16_t;
        using AddressByte = uint8_t;
        using AddressWord = uint16_t;

        struct Opcode
        {
            const enum class OPCODE {
                MOVE_BYTE,
                STORE_BYTE,
                LOAD_BYTE,
                MEM_HIGH,
                MEM_MOVE,
                LOAD_WORD,
                STORE_STACK_POINTER,
                MOVE_STACK_POINTER,
                INC_WORD,
                DEC_WORD,
                ADD_HL,
                ADD_STACK_POINTER,
                BIN_OP_REG,
                BIN_OP_MEM,
                PUSH,
                POP,
                JUMP_RELATIVE,
                JUMP_IMMEDIATE,
                CALL,
                RET,
                RST,
                ROT,
                BIT,
                BYTE
            } type;
            Opcode(OPCODE type) : type{type} {}
            virtual ~Opcode() = default;
            virtual std::string ToString() const = 0;
            virtual size_t write(std::ostream &) const = 0;

            virtual RegisterCobbling CobbledRegisters() const
            {
                return RegisterCobbling{.byte = 0};
            }

            virtual FlagCobbling FlagBehavior() const
            {
                return {FlagAction::IGNORE, FlagAction::IGNORE, FlagAction::IGNORE, FlagAction::IGNORE};
            }
        };

        struct MoveByte : public Opcode
        {
            MoveByte(RegisterByte dst, std::variant<RegisterByte, ImmediateByte> src) : Opcode{OPCODE::MOVE_BYTE}, dst{dst}, src{src} {}
            ~MoveByte() override = default;
            const RegisterByte dst;
            const std::variant<RegisterByte, ImmediateByte> src;

            std::string ToString() const override
            {
                std::string src_str = std::holds_alternative<RegisterByte>(src) ? register_byte_names[e2i(std::get<RegisterByte>(src))] : std::to_string(std::get<ImmediateByte>(src));
                return "LD " + register_byte_names[e2i(dst)] + "," + src_str;
            }

            size_t write(std::ostream &stream) const override
            {
                if (const RegisterByte *src_ptr = std::get_if<RegisterByte>(&src))
                {
                    uint8_t byte = 0x40 | (e2i(dst) << 3) | e2i(*src_ptr);
                    WriteByte(stream, byte);
                    return 1;
                }
                uint8_t bytes[]{0x06 | (e2i(dst) << 3), std::get<ImmediateByte>(src)};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                if (const RegisterByte *dst_ptr = std::get_if<RegisterByte>(&src))
                {
                    if (*dst_ptr == dst)
                    {
                        return RegisterCobbling{.byte = 0};
                    }
                }
                return cobble_units[e2i(dst)];
            }
        };

        struct StoreByte : public Opcode
        {
            StoreByte(RegisterPointer dst) : Opcode{OPCODE::STORE_BYTE}, dst{dst} {}
            ~StoreByte() override = default;
            const RegisterPointer dst;

            std::string ToString() const override
            {
                return "LD (" + register_pointer_names[e2i(dst)] + "),A";
            }

            size_t write(std::ostream &stream) const override
            {
                static uint8_t opcodes[]{0x02, 0x12, 0x22, 0x32, 0xE2};
                WriteByte(stream, opcodes[e2i(dst)]);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                if (dst == RegisterPointer::HL_DEC || dst == RegisterPointer::HL_INC)
                {
                    return RegisterCobbling{.byte = static_cast<uint8_t>(cobble_units[e2i(RegisterByte::H)].byte | cobble_units[e2i(RegisterByte::L)].byte)};
                }
                return RegisterCobbling{.byte = 0};
            }
        };

        struct LoadByte : public Opcode
        {
            LoadByte(RegisterPointer src) : Opcode{OPCODE::LOAD_BYTE}, src{src} {}
            ~LoadByte() override = default;
            const RegisterPointer src;

            std::string ToString() const override
            {
                return "LD A,(" + register_pointer_names[e2i(src)] + ")";
            }

            size_t write(std::ostream &stream) const override
            {
                static uint8_t opcodes[]{0x0A, 0x1A, 0x2A, 0x3A, 0xF2};
                WriteByte(stream, opcodes[e2i(src)]);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                if (src == RegisterPointer::HL_DEC || src == RegisterPointer::HL_INC)
                {
                    return RegisterCobbling{.byte = static_cast<uint8_t>(cobble_units[e2i(RegisterByte::H)].byte | cobble_units[e2i(RegisterByte::L)].byte)};
                }
                return RegisterCobbling{.byte = 0};
            }
        };

        struct MemHigh : public Opcode
        {
            MemHigh(ImmediateByte address, bool store) : Opcode{OPCODE::MEM_HIGH}, address{address}, store{store} {}
            ~MemHigh() override = default;
            ImmediateByte address;
            bool store;

            std::string ToString() const override
            {
                return store ? ("LDH (" + std::to_string(address) + "),A") : "LDH A,(" + std::to_string(address) + ")";
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[]{store ? 0xE0 : 0xF0, address};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return store ? RegisterCobbling{.byte = 0} : cobble_units[e2i(RegisterByte::A)];
            }
        };

        struct MemMove : public Opcode
        {
            MemMove(ImmediateWord address, bool store) : Opcode{OPCODE::MEM_MOVE}, address{address}, store{store} {}
            ~MemMove() override = default;
            ImmediateWord address;
            bool store;

            std::string ToString() const override
            {
                return store ? ("LD (" + std::to_string(address) + "),A") : "LD A,(" + std::to_string(address) + ")";
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = store ? 0xEA : 0xFA;
                WriteByte(stream, byte);
                WriteWordLE(stream, address);
                return 3;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return store ? RegisterCobbling{.byte = 0} : cobble_units[e2i(RegisterByte::A)];
            }
        };

        struct LoadWord : public Opcode
        {
            LoadWord(RegisterWord dst, ImmediateWord src) : Opcode{OPCODE::LOAD_WORD}, dst{dst}, src{src} {}
            ~LoadWord() override = default;
            const RegisterWord dst;
            const ImmediateWord src;

            std::string ToString() const override
            {
                return "LD " + register_word_names[e2i(dst)] + "," + std::to_string(src);
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x01 | (e2i(dst) << 4);
                WriteByte(stream, byte);
                WriteWordLE(stream, src);
                return 3;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (dst == RegisterWord::SP_AF) ? RegisterCobbling{.sp = 1} : cobble_words[e2i(dst)];
            }
        };

        struct StoreStackPointer : public Opcode
        {
            StoreStackPointer(AddressWord addr) : Opcode{OPCODE::STORE_STACK_POINTER}, dst{addr} {}
            ~StoreStackPointer() override = default;
            const AddressWord dst;

            std::string ToString() const override
            {
                return "LD (" + std::to_string(dst) + "),SP";
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x08;
                WriteByte(stream, byte);
                WriteWordLE(stream, dst);
                return 3;
            }
        };

        struct MoveStackPointer : public Opcode
        {
            ~MoveStackPointer() override = default;
            const ImmediateByte offset;

            std::string ToString() const override
            {
                return "LD HL,SP+" + std::to_string(offset);
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[] = {0xF8, offset};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return cobble_words[e2i(RegisterWord::HL)];
            }

            FlagCobbling FlagBehavior() const override
            {
                return {FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN, FlagAction::UNKNOWN};
            }
        };

        struct IncWord : public Opcode
        {
            ~IncWord() override = default;
            const RegisterWord reg;

            std::string ToString() const override
            {
                return "INC " + register_word_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x03 | (e2i(reg) << 4);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterWord::SP_AF) ? RegisterCobbling{.sp = 1} : cobble_words[e2i(reg)];
            }
        };

        struct DecWord : public Opcode
        {
            ~DecWord() override = default;
            const RegisterWord reg;

            std::string ToString() const override
            {
                return "DEC " + register_word_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x0B | (e2i(reg) << 4);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterWord::SP_AF) ? RegisterCobbling{.sp = 1} : cobble_words[e2i(reg)];
            }
        };

        struct IncByte : public Opcode
        {
            ~IncByte() override = default;
            const RegisterByte reg;

            std::string ToString() const override
            {
                return "INC " + register_byte_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x04 | (e2i(reg) << 3);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterByte::PTR_HL) ? RegisterCobbling{.byte = 0} : cobble_units[e2i(reg)];
            }

            FlagCobbling FlagBehavior() const override
            {
                return {FlagAction::UNKNOWN, FlagAction::CLEAR, FlagAction::UNKNOWN, FlagAction::IGNORE};
            }
        };

        struct DecByte : public Opcode
        {
            ~DecByte() override = default;
            const RegisterByte reg;

            std::string ToString() const override
            {
                return "DEC " + register_byte_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x05 | (e2i(reg) << 3);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterByte::PTR_HL) ? RegisterCobbling{.byte = 0} : cobble_units[e2i(reg)];
            }

            FlagCobbling FlagBehavior() const override
            {
                return {FlagAction::UNKNOWN, FlagAction::SET, FlagAction::UNKNOWN, FlagAction::IGNORE};
            }
        };

        struct AddHl : public Opcode
        {
            ~AddHl() override = default;
            const RegisterWord src;

            std::string ToString() const override
            {
                return "ADD HL," + register_word_names[e2i(src)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x09 | (e2i(src) << 4);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return cobble_words[e2i(RegisterWord::HL)];
            }

            FlagCobbling FlagBehavior() const override
            {
                return {FlagAction::IGNORE, FlagAction::CLEAR, FlagAction::UNKNOWN, FlagAction::UNKNOWN};
            }
        };

        struct AddStackPointer : public Opcode
        {
            ~AddStackPointer() override = default;
            const ImmediateByte src;

            std::string ToString() const override
            {
                return "ADD SP," + std::to_string(src);
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[]{0xE8, src};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return cobble_words[e2i(RegisterWord::SP_AF)];
            }

            FlagCobbling FlagBehavior() const override
            {
                return {FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN, FlagAction::UNKNOWN};
            }
        };

        struct BinOpReg : public Opcode
        {
            BinOpReg(BinOp op, RegisterByte src) : Opcode{OPCODE::BIN_OP_REG}, op{op}, src{src} {}
            ~BinOpReg() override = default;
            const BinOp op;
            const RegisterByte src;

            std::string ToString() const override
            {
                return bin_op_names[e2i(op)] + " A," + register_byte_names[e2i(src)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0x80 | (e2i(op) << 3) | e2i(src);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return cobble_units[e2i(RegisterByte::A)];
            }

            FlagCobbling FlagBehavior() const override
            {
                FlagAction zero = FlagAction::UNKNOWN;
                FlagAction subtract = (op == BinOp::SUB || op == BinOp::SBC || op == BinOp::CP) ? FlagAction::SET : FlagAction::CLEAR;
                FlagAction half = (op == BinOp::OR || op == BinOp::XOR) ? FlagAction::CLEAR : (op == BinOp::AND ? FlagAction::SET : FlagAction::UNKNOWN);
                FlagAction carry = (op == BinOp::AND || op == BinOp::XOR || op == BinOp::OR) ? FlagAction::CLEAR : FlagAction::UNKNOWN;
                return {zero, subtract, half, carry};
            }
        };

        struct BinOpImm : public Opcode
        {
            ~BinOpImm() override = default;
            const BinOp op;
            const ImmediateByte src;

            std::string ToString() const override
            {
                return bin_op_names[e2i(op)] + " A," + std::to_string(src);
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[]{0xC6 | (e2i(op) << 3), src};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return cobble_units[e2i(RegisterByte::A)];
            }

            FlagCobbling FlagBehavior() const override
            {
                FlagAction zero = FlagAction::UNKNOWN;
                FlagAction subtract = (op == BinOp::SUB || op == BinOp::SBC || op == BinOp::CP) ? FlagAction::SET : FlagAction::CLEAR;
                FlagAction half = (op == BinOp::OR || op == BinOp::XOR) ? FlagAction::CLEAR : (op == BinOp::AND ? FlagAction::SET : FlagAction::UNKNOWN);
                FlagAction carry = (op == BinOp::AND || op == BinOp::XOR || op == BinOp::OR) ? FlagAction::CLEAR : FlagAction::UNKNOWN;
                return {zero, subtract, half, carry};
            }
        };

        struct Push : public Opcode
        {
            ~Push() override = default;
            const RegisterWord reg;

            std::string ToString() const override
            {
                return "PUSH " + push_pop_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0xC5 | (e2i(reg) << 4);
                WriteByte(stream, byte);
                return 1;
            }
        };

        struct Pop : public Opcode
        {
            ~Pop() override = default;
            const RegisterWord reg;

            std::string ToString() const override
            {
                return "POP " + push_pop_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t byte = 0xC1 | (e2i(reg) << 4);
                WriteByte(stream, byte);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterWord::SP_AF) ? cobble_units[e2i(RegisterByte::A)] : cobble_words[e2i(reg)];
            }

            FlagCobbling FlagBehavior() const override
            {
                if (reg == RegisterWord::SP_AF)
                {
                    return {FlagAction::UNKNOWN, FlagAction::UNKNOWN, FlagAction::UNKNOWN, FlagAction::UNKNOWN};
                }
                return {FlagAction::IGNORE, FlagAction::IGNORE, FlagAction::IGNORE, FlagAction::IGNORE};
            }
        };

        struct JumpRelative : public Opcode
        {
            ~JumpRelative() override = default;
            const Condition condition;
            const ImmediateByte offset;

            std::string ToString() const override
            {
                return "JR " + condition_names[e2i(condition)] + std::to_string(*reinterpret_cast<const char *>(&offset));
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[]{0x18 | (e2i(condition) << 3), offset};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }
        };

        struct JumpImmediate : public Opcode
        {
            JumpImmediate(Condition condition, ImmediateWord address) : Opcode{OPCODE::JUMP_IMMEDIATE}, condition{condition}, address{address} {}
            ~JumpImmediate() override = default;
            const Condition condition;
            const ImmediateWord address;

            std::string ToString() const override
            {
                return "JP " + condition_names[e2i(condition)] + std::to_string(address);
            }

            size_t write(std::ostream &stream) const override
            {
                WriteByte(stream, (condition == Condition::ALWAYS) ? 0xC3 : (0xC2 | ((e2i(condition) - 1) << 3)));
                WriteWordLE(stream, address);
                return 3;
            }
        };

        struct CallSymbol : public Opcode
        {
            ~CallSymbol() override = default;
            const Condition condition;
            const std::string symbol;

            std::string ToString() const override
            {
                return "CALL " + condition_names[e2i(condition)] + symbol;
            }

            size_t write(std::ostream &stream) const override
            {
                std::string msg = "Call to symbol \"" + symbol + "\" has not been resolved";
                throw msg;
            }
        };

        struct Call : public Opcode
        {
            Call(Condition condition, ImmediateWord address) : Opcode{OPCODE::CALL}, condition{condition}, address{address} {}
            ~Call() override = default;
            const Condition condition;
            const ImmediateWord address;

            std::string ToString() const override
            {
                return "CALL " + condition_names[e2i(condition)] + std::to_string(address);
            }

            size_t write(std::ostream &stream) const override
            {
                WriteByte(stream, (condition == Condition::ALWAYS) ? 0xCD : (0xC4 | ((e2i(condition) - 1) << 3)));
                WriteWordLE(stream, address);
                return 3;
            }
        };

        struct Ret : public Opcode
        {
            ~Ret() override = default;
            const Condition condition;

            std::string ToString() const override
            {
                return "RET " + condition_names[e2i(condition)];
            }

            size_t write(std::ostream &stream) const override
            {
                WriteByte(stream, (condition == Condition::ALWAYS) ? 0xC9 : (0xC0 | ((e2i(condition) - 1) << 3)));
                return 1;
            }
        };

        struct Rst : public Opcode
        {
            ~Rst() override = default;
            const RstVector rst;

            std::string ToString() const override
            {
                return "RST " + std::to_string(e2i(rst) * 8);
            }

            size_t write(std::ostream &stream) const override
            {
                WriteByte(stream, 0xC7 | (e2i(rst) << 3));
                return 1;
            }
        };

        struct RotOpReg : public Opcode
        {
            RotOpReg(RotOp op, RegisterByte reg) : Opcode{OPCODE::ROT}, op{op}, reg{reg} {}
            ~RotOpReg() override = default;
            const RotOp op;
            const RegisterByte reg;

            std::string ToString() const override
            {
                return rot_op_names[e2i(op)] + " " + register_byte_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[]{0xCB, (e2i(op) << 3) | e2i(reg)};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterByte::PTR_HL) ? RegisterCobbling{.byte = 0} : cobble_units[e2i(reg)];
            }

            FlagCobbling FlagBehavior() const override
            {
                FlagAction carry = (op == RotOp::SWAP) ? FlagAction::CLEAR : FlagAction::UNKNOWN;
                return {FlagAction::UNKNOWN, FlagAction::CLEAR, FlagAction::CLEAR, carry};
            }
        };

        struct BitOpReg : public Opcode
        {
            BitOpReg(BitOp op, Bit bit, RegisterByte reg) : Opcode{OPCODE::BIT}, op{op}, bit{bit}, reg{reg} {}
            ~BitOpReg() override = default;
            const BitOp op;
            const Bit bit;
            const RegisterByte reg;

            std::string ToString() const override
            {
                return bit_op_names[e2i(op)] + " " + std::to_string(e2i(bit)) + "," + register_byte_names[e2i(reg)];
            }

            size_t write(std::ostream &stream) const override
            {
                uint8_t bytes[]{0xCB, 0x40 | (e2i(op) << 6) | (e2i(bit) << 3) | e2i(reg)};
                stream.write(reinterpret_cast<char *>(bytes), 2);
                return 2;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return (reg == RegisterByte::PTR_HL) ? RegisterCobbling{.byte = 0} : cobble_units[e2i(reg)];
            }

            FlagCobbling FlagBehavior() const override
            {
                return (op == BitOp::BIT) ? FlagCobbling{FlagAction::UNKNOWN, FlagAction::CLEAR, FlagAction::SET, FlagAction::IGNORE} : FlagCobbling{FlagAction::IGNORE, FlagAction::IGNORE, FlagAction::IGNORE, FlagAction::IGNORE};
            }
        };

        struct ByteOpcode : public Opcode
        {
            ByteOpcode(ByteOp op) : Opcode{OPCODE::BYTE}, op{op} {}
            ~ByteOpcode() override = default;
            const ByteOp op;

            std::string ToString() const override
            {
                return byte_ops[e2i(op)].name;
            }

            size_t write(std::ostream &stream) const override
            {
                WriteByte(stream, byte_ops[e2i(op)].opcode);
                return 1;
            }

            RegisterCobbling CobbledRegisters() const override
            {
                return byte_ops[e2i(op)].register_cobbling;
            }

            FlagCobbling FlagBehavior() const override
            {
                return byte_ops[e2i(op)].flag_cobbling;
            }
        };

    } // namespace gbops

    namespace linker
    {

        struct Patch
        {
            const std::string_view base;
            const size_t offset;

            bool operator==(const Patch &other) const
            {
                return offset == other.offset && base == other.base;
            }
        };

        struct OpcodeFunction
        {
            const std::string_view name;
            const std::vector<std::unique_ptr<gbops::Opcode>> opcodes;
            const std::map<size_t, std::string_view> labels;
            const std::map<size_t, Patch> patches;

            OpcodeFunction OptimizeBlind();
        };

        struct LinkedFunction
        {
            const std::vector<uint8_t> data;
            const std::map<size_t, Patch> patches;

            bool operator==(const LinkedFunction &other) const
            {
                return data == other.data && patches == other.patches;
            }
        };

        #pragma pack(push, 1)
        struct Header
        {

            enum class MapperType : uint8_t
            {
                ROM,
                MBC1,
                MBC1_RAM,
                MBC1_RAM_BATTERY,
                MBC2 = 0x05,
                MBC2_BATTERY,
                ROM_RAM = 0x08,
                ROM_RAM_BATTERY,
                MMM01 = 0x0B,
                MMM01_RAM,
                MMM01_RAM_BATTERY,
                MBC3_TIMER_BATTERY = 0x0F,
                MBC3_TIMER_RAM_BATTERY,
                MBC3,
                MBC3_RAM,
                MBC3_RAM_BATTERY,
                MBC5 = 0x19,
                MBC5_RAM,
                MBC5_RAM_BATTERY,
                MBC5_RUMBLE,
                MBC5_RUMBLE_RAM,
                MBC5_RUMBLE_RAM_BATTERY,
                MBC6 = 0x20,
                MBC7_SENSOR_RUMBLE_RAM_BATTERY = 0x22,
                POCKET_CAMERA = 0xFC,
                BANDAI_TAMA5,
                HuC3,
                HuC1_RAM_BATTERY
            };
            enum class CgbType : uint8_t
            {
                BACKWARDS_COMPATIBLE = 0x80,
                CGB_ONLY = 0xC0
            };
            enum class RamBanks : uint8_t { // 8KiB each
                NONE,
                ONE = 2,
                FOUR,
                SIXTEEN,
                EIGHT
            };
            enum class DestinationCode : uint8_t {
                JAPAN,
                INTL
            };
            uint16_t starting_point;
            union
            {
                struct
                {
                    char short_title[11];
                    char manufacturer[4];
                    CgbType cgb_type;
                };
                char long_title[16];
            };
            char new_licensee[2];
            MapperType mapper_type;
            uint8_t rom_size_exp;
            RamBanks num_ram_banks;
            DestinationCode destination_code;
            uint8_t old_licensee;
            uint8_t version;

            uint8_t Checksum() const {
                uint8_t checksum = 0;
                for (auto byte : long_title) {
                    checksum -= (byte & 0xFF) + 1;
                }
                for (auto byte : new_licensee) {
                    checksum -= (byte & 0xFF) + 1;
                }
                checksum -= e2i(mapper_type) + 1;
                checksum -= rom_size_exp + 1;
                checksum -= e2i(num_ram_banks) + 1;
                checksum -= e2i(destination_code) + 1;
                checksum -= old_licensee + 1;
                checksum -= version;
                return checksum;
            }
        };
        #pragma pack(pop)

        class Linker
        {
        private:
            /// @brief Translate a contiguous series of instructions into binary code without backpatching. Saves offsets from start of function to a map of symbols.
            /// @param function_name In: Name of function, used for symbol resolution
            /// @param instrs In: Instructions to link
            /// @param labels_in In: Maps each internal label to the opcode number it precedes
            /// @param patches_in In: Maps number of opcode to symbols
            /// @param buffer Out: Destination to which to save binary
            /// @param patches_out Out: Associates offsets from start of function in bytes with addresses to which they shall point
            /// @return Size of linked function in bytes
            size_t LinkFunction_(const std::string_view function_name, const std::span<const std::unique_ptr<gbops::Opcode>> instrs, const std::map<size_t, std::string_view> &labels_in, const std::map<size_t, Patch> &patches_in, std::ostream &buffer, std::map<size_t, Patch> &patches_out);

            /// @brief Write the unpatched binary of a function to a buffer and accumulate the adjusted patch locations.
            /// @param data
            /// @param patches_in
            /// @param start_offset In: The position, in bytes, of the start of the function in ROM
            /// @param buffer
            /// @param patch_accum
            void WriteFunction_(const std::span<const uint8_t> data, const std::map<size_t, Patch> &patches_in, size_t start_offset, std::ostream &buffer, std::map<size_t, Patch> &patch_accum);

            /// @brief Backpatch symbol pointers in a buffering stream
            /// @param patches 
            /// @param symbols 
            /// @param start_offset In: Number of bytes to subtract from patch position - i.e. bytes written before the start of stream
            /// @param stream 
            void BackPatch_(const std::map<size_t, Patch> &patches, const std::map<std::string_view, gbops::AddressWord> &symbols, size_t start_offset, std::ostream &stream);

            /// @brief Writes a header up to the header checksum. Does not write the global checksum
            /// @param header 
            /// @param stream Should be at position 0x100
            void WriteHeader_(const Header& header, std::ostream& stream) const;

            void WriteChecksum_(std::iostream& stream) const;

        public:
            LinkedFunction LinkFunction(const OpcodeFunction &func);

            /// @brief Write the header and collection of functions
            /// @param header 
            /// @param symbols Includes functions AND global vars
            /// @param functions This MAY also end up including read-only ROM data
            /// @param stream Output. Will be positioned at end of ROM at function's return
            void Write(const Header& header, const std::map<std::string_view, gbops::AddressWord> &symbols, const std::map<std::string_view, LinkedFunction>& functions, std::iostream &stream);
        };

    } // namespace linker

} // namespace gbds

#undef e2i

#endif // _LINKER_HPP
