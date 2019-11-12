
const REGISTRY : [Entity; EntityNum::Max as usize] = [
    Entity::Section,
    Entity::Register(Register { dummy : 0 }),
    Entity::Register(Register { dummy : 1 }),
    Entity::Register(Register { dummy : 2 }),
];

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug)]
enum EntityNum {
    test,
    test_echo,
    test_reg,
    test_num,
    Max,
}

const STR_TO_ENUM : phf::Map<&'static str, EntityNum> = phf_map! {
    "test"      => EntityNum::test,
    "test/echo" => EntityNum::test_echo,
    "test/reg"  => EntityNum::test_reg,
    "test/num"  => EntityNum::test_num,
};