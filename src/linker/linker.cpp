#include "linker.hpp"

#include <sstream>

#define e2i(x) static_cast<int>(x)

namespace gbds
{
    namespace gbops {
        

        std::string register_byte_names[]{"B", "C", "D", "E", "H", "L", "(HL)", "A"};

        std::string register_pointer_names[]{"BC", "DE", "HL+", "HL-", "(C)"};

        std::string register_word_names[]{"BC", "DE", "HL", "SP"};
        std::string push_pop_names[]{"BC", "DE", "HL", "AF"};

        std::string bin_op_names[]{"ADD", "ADC", "SUB", "SBC", "AND", "XOR", "OR", "CP"};

        std::string rot_op_names[]{"RLC", "RRC", "RL", "RR", "SLA", "SRA", "SWAP", "SRL"};

        std::string bit_op_names[]{"BIT", "RES", "SET"};

        std::string condition_names[]{"", "NZ,", "Z,", "NC,", "C,"};

        RegisterCobbling cobble_units[]{{.b = 1}, {.c = 1}, {.d = 1}, {.e = 1}, {.h = 1}, {.l = 1}, {.sp = 1}, {.a = 1}};
        RegisterCobbling cobble_words[]{
            {.byte = cobble_units[e2i(RegisterByte::B)].byte | cobble_units[e2i(RegisterByte::C)].byte},
            {.byte = cobble_units[e2i(RegisterByte::D)].byte | cobble_units[e2i(RegisterByte::E)].byte},
            {.byte = cobble_units[e2i(RegisterByte::H)].byte | cobble_units[e2i(RegisterByte::L)].byte}};

        ByteOpT byte_ops[]{
            {"NOP", 0x00, {.byte = 0}, {FlagAction::IGNORE}},
            {"STOP", 0x10, {.byte = 0}, {FlagAction::IGNORE}},
            {"HALT", 0x76, {.byte = 0}, {FlagAction::IGNORE}},
            {"DI", 0xF3, {.byte = 0}, {FlagAction::IGNORE}},
            {"EI", 0xFB, {.byte = 0}, {FlagAction::IGNORE}},
            {"RLCA", 0x07, cobble_units[e2i(RegisterByte::A)], {FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN}},
            {"RRCA", 0x0F, cobble_units[e2i(RegisterByte::A)], {FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN}},
            {"RLA", 0x17, cobble_units[e2i(RegisterByte::A)], {FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN}},
            {"RRA", 0x1F, cobble_units[e2i(RegisterByte::A)], {FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN}},
            {"DAA", 0x27, cobble_units[e2i(RegisterByte::A)], {FlagAction::UNKNOWN, FlagAction::IGNORE, FlagAction::CLEAR, FlagAction::UNKNOWN}},
            {"CPL", 0x2F, cobble_units[e2i(RegisterByte::A)], {FlagAction::IGNORE, FlagAction::SET, FlagAction::SET, FlagAction::IGNORE}},
            {"SCF", 0x37, cobble_units[e2i(RegisterByte::A)], {FlagAction::IGNORE, FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::SET}},
            {"CCF", 0x3F, cobble_units[e2i(RegisterByte::A)], {FlagAction::IGNORE, FlagAction::CLEAR, FlagAction::CLEAR, FlagAction::UNKNOWN}},
            {"JP HL", 0xE9, {.byte = 0}, {FlagAction::IGNORE}},
            {"LD SP,HL", 0xF9, cobble_units[e2i(RegisterByte::PTR_HL)], {FlagAction::IGNORE}}};
    }

    namespace linker
    {

        OpcodeFunction OpcodeFunction::OptimizeBlind() {
            using namespace gbops;
            std::vector<std::unique_ptr<Opcode>> new_opcodes;
            std::map<size_t, std::string_view> new_labels;
            std::map<size_t, Patch> new_patches;
            // TODO optimize individual opcodes
            // return {std::string(name), std::move(new_opcodes), std::move(new_labels), std::move(new_patches)};
            return *this;
        }

        size_t Linker::LinkFunction_(const std::string_view function_name, const std::span<const std::unique_ptr<gbops::Opcode>> ops, const std::map<size_t, std::string_view> &labels_in, const std::map<size_t, Patch> &patches_in, std::ostream &buffer, std::map<size_t, Patch> &patches_out)
        {
            size_t offset = 0;
            size_t idx = 0;
            std::map<std::string_view, size_t> local_labels;
            std::map<size_t, Patch> local_patches;
            for (const std::unique_ptr<gbops::Opcode> &opcode : ops)
            {
                // Write opcode
                size_t oplen = opcode->write(buffer);

                // Save offset of label
                for (const auto &itr = labels_in.find(idx); itr != labels_in.end();)
                {
                    local_labels.insert({itr->second, offset});
                    break;
                }

                // Save offset and target of patch
                for (const auto &itr = patches_in.find(idx); itr != patches_in.end();)
                {
                    // Patches will always be the 2nd 2 bytes of a 3-byte opcode
                    local_patches.insert({offset + 1, itr->second});
                    break;
                }
                offset += oplen;
                idx++;
            }

            // Translate local patches to output patches
            for (const auto &itr : local_patches)
            {
                size_t pos = itr.first;
                const std::string_view key = itr.second.base;
                const auto &local_itr = local_labels.find(key);
                if (local_itr == local_labels.end())
                {
                    // Patch does not point to label within function
                    patches_out.insert({pos, itr.second});
                }
                else
                {
                    // Patch points to local label within function
                    patches_out.insert({pos, {function_name, local_itr->second + itr.second.offset}});
                }
            }

            // Return number of bytes written
            return offset;
        }

        LinkedFunction Linker::LinkFunction(const OpcodeFunction &func)
        {
            std::stringstream sstr;
            std::map<size_t, Patch> patches;
            size_t size = LinkFunction_(func.name, func.opcodes, func.labels, func.patches, sstr, patches);
            if (size != sstr.tellp())
                throw "Messed up the byte count somewhere";
            std::vector<uint8_t> binary(size);
            sstr.read(reinterpret_cast<char *>(binary.data()), size);
            return LinkedFunction{std::move(binary), std::move(patches)};
        }

        void Linker::WriteFunction_(const std::span<const uint8_t> binary, const std::map<size_t, Patch>& patches_in, size_t start_offset, std::ostream& buffer, std::map<size_t, Patch>& patch_accum) {
            buffer.write(reinterpret_cast<const char*>(binary.data()), binary.size());
            for (const auto& itr : patches_in) {
                patch_accum.insert({itr.first + start_offset, itr.second});
            }
        }

        void Linker::BackPatch_(const std::map<size_t, Patch>& patches, const std::map<std::string_view, gbops::AddressWord>& symbols, size_t start_offset, std::ostream& stream) {
            for (const auto& itr : patches) {
                stream.seekp(itr.first - start_offset);
                const auto& sym_itr = symbols.find(itr.second.base);
                if (sym_itr == symbols.end()) {
                    throw "Could not find symbol \"" + std::string(itr.second.base) + "\"";
                }
                gbops::AddressWord value = sym_itr->second + itr.second.offset;
                WriteWordLE(stream, value);
            }
        }

        void Linker::WriteHeader_(const Header& header, std::ostream& stream) const {
            // NOP; JP <starting_point>;
            WriteByte(stream, 0x00); // 0x101
            WriteByte(stream, 0xC3); // 0x102
            WriteWordLE(stream, header.starting_point); // 0x104

            char zeros[48] {0}; // Logo placeholder
            stream.write(zeros, sizeof(zeros)); // 0x134
            stream.write(reinterpret_cast<const char*>(&header.long_title), 25); // 0x14D
            uint8_t checksum = header.Checksum(); // 0x14E
            WriteByte(stream, checksum);
            // Does not write global checksum
        }

        void Linker::WriteChecksum_(std::iostream& stream) const {
            stream.seekg(0);
            char c;
            uint16_t checksum = 0;
            while((c = stream.get()) != EOF) {
                checksum += c & 0xff;
            }
            stream.clear(); // Clear EOF bit
            stream.seekp(0x14E);
            WriteWordLE(stream, checksum);
        }

        // Fill a ROM up to a certain size
        static void pad_to(std::ostream& stream, size_t len, char fill = 0) {
            stream.seekp(0, std::ios::end);
            size_t pos = stream.tellp();
            if (pos >= len) {
                stream.seekp(pos);
                return;
            }
            size_t remaining = len - pos;
            std::vector<char> zeros(remaining, fill);
            stream.write(zeros.data(), remaining);
        }

        void Linker::Write(const Header& header, const std::map<std::string_view, gbops::AddressWord>& symbols, const std::map<std::string_view, LinkedFunction>& funcs, std::iostream& stream) {
            // Write header
            stream.seekp(0);
            pad_to(stream, 0x100);
            WriteHeader_(header, stream);

            // Write functions
            std::map<size_t, Patch> patches;
            for (const auto& itr : funcs) {
                std::string_view name = itr.first;
                const LinkedFunction& function = itr.second;
                const auto& sym_itr = symbols.find(name);
                if (sym_itr == symbols.end()) {
                    throw "No location found for function \"" + std::string(name) + "\"";
                }
                gbops::AddressWord addr = sym_itr->second;
                // TODO: handle ROM banks
                pad_to(stream, addr);
                WriteFunction_(function.data, function.patches, addr, stream, patches);
            }
            
            // Back patch references
            BackPatch_(patches, symbols, 0, stream);

            // Write remaining padding
            size_t total_rom_size = 1 << (15 + header.rom_size_exp);
            pad_to(stream, total_rom_size);

            // Write global checksum
            WriteChecksum_(stream);
        }

    } // namespace linker

} // namespace gbds

#undef e2i
