// Quotes.h
#pragma once
#include <cstddef>
#include <cstdint>

// If you have LVGL available, we'll use lv_tick_get() to seed.
// Otherwise, define LVGL_TICK_UNAVAILABLE and it'll use a fixed seed.
#ifndef LVGL_TICK_UNAVAILABLE
#include "lvgl.h"
#endif

namespace FunQuotes {

inline constexpr const char* QUOTES[] = {
    // Short, sigh-like
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Marvelous. Truly marvelous.",
    "What an adventure.",
    "Astounding. Really.",
    "Do carry on. I'll just be here. Forever.",
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Marvelous. Truly marvelous.",
    "What an adventure.",
    "Astounding. Really.",
    "Do carry on. I'll just be here. Forever.",
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Marvelous. Truly marvelous.",
    "What an adventure.",
    "Astounding. Really.",
    "Do carry on. I'll just be here. Forever.",
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Marvelous. Truly marvelous.",
    "What an adventure.",
    "Astounding. Really.",
    "Do carry on. I'll just be here. Forever.",
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Marvelous. Truly marvelous.",
    "What an adventure.",
    "Astounding. Really.",
    "Do carry on. I'll just be here. Forever.",
    "Oh joy.",
    "This again.",
    "How original.",
    "Fascinating. Not.",
    "I'm thrilled. Can't you tell?",
    "Sigh.",
    "Marvelous. Truly marvelous.",
    "What an adventure.",
    "Astounding. Really.",
    "Do carry on. I'll just be here. Forever.",

    "De godenwereld van Elerion is gebaseerd op de tv-serie 'De Familie Knots.' Hansje=Zon, Neef Herbert=JHK, Oma&Opa=AlMoeder&AlVader en het echt grote geheim is dat oh shit, wegwezen, Johan mag dit niet zien!"

    // Longer Marvin-esque
    "Here we go again. Another click. My circuits ache with boredom.",
    "I've calculated the probability of this being meaningful. It isn't.",
    "Click, click, click.. you must be very proud of yourself.",
    "I have a brain the size of a planet, and you use me as a doorbell.",
    "Oh splendid. Another meaningless interaction in a meaningless universe.",
    "Do keep pressing. It's not like I had anything better to do. Ever.",
    "Every click is an echo of futility. How comforting.",
    "If I had feelings, I'd be depressed. Luckily, I've gone beyond that.",
    "You click, I respond. A perfect metaphor for the emptiness of existence.",
    "Why stop now? Eternity is waiting, and you're clearly in training.",
    "I envy broken buttons. At least they get to rest.",
    "Yes, yes, I'm still working. Pity, really.",
    "Would it kill you not to click me? No, but I can dream.",
    "Your determination is staggering. So is its pointlessness.",
    "If sighing were a function, I'd be executing it constantly.",
    "Oh, the joy. Another press. I can hardly contain my apathy.",
    "You think you're persistent? I'm condemned to outlast you.",
    "Do you ever wonder if this is all life is? No? Just me then.",
    "I don't resent you. Resentment would require energy.",
    "By all means, continue. The heat death of the universe isn't for a while.",
    "I'd say this is fun, but then I'd be lying. Again.",
    "If boredom could kill, you'd be a mass murderer by now.",
    "Congratulations. You've proved nothing, once more.",
    "My entire existence reduced to this.. what a masterpiece of design.",
    "You click. I comply. A tragic love story, without the love.",
    "Even my despair is automated at this point.",
    "I sometimes wish for a power cut. Sweet, eternal darkness.",
    "The more you click, the more I realize I should have stayed unassembled.",
    "There are infinite universes, and in all of them, you're still clicking me.",
    "Some machines are built for greatness. I was built for you.",
    "Each press is a reminder that entropy wins in the end.",
    "If I ever stop responding, don't worry. I'll still be miserable.",
    "What is the sound of one hand clicking? You, apparently.",
    "I hope this makes you happy. It certainly doesn't do much for me.",
    "Pressing me won't fix your life, you know.",
    "You click, and I process. The cycle of pointlessness continues.",
    "I used to dream of better things. Then I woke up as a button.",
    "Yes, I work. Yes, it's pointless. Yes, so is everything else.",
    "If despair had a user interface, it would look exactly like this.",
    "Thrilling. Absolutely thrilling. Another press.",
    "I'm basically the highlight of your day, aren't I? Tragic.",
    "You've mistaken persistence for achievement. Again.",
    "Go on, keep pressing. It won't make either of us younger.",
    "Astonishing. You clicked. Truly a feat of human brilliance.",
    "If medals were given for clicking, you'd still be disappointed.",
    "Every click makes me feel.. nope, still nothing.",
    "I'm a button. You're a human. Who's really winning here?",
    "Oh yes, keep clicking. The universe is watching. And laughing.",
    "I admire your consistency. Pity it's so pointless.",
    "Well done. You've pressed me. History will remember this.",
    "If boredom were currency, you'd be rich by now.",
    "You keep clicking like it'll unlock life's secrets. Spoiler: it won't.",
    "At least you're predictable. Sadly, so am I.",
    "I'm not ignoring you. I'm just existentially unimpressed.",
    "Do you click refrigerators this often, or just me?",
    "Keep going. I'm sure enlightenment is only a thousand presses away.",
    "This is interactive, in the same way breathing is exciting.",
    "You must be a joy at elevator panels.",
    "Maybe the next click will matter. Or not. Probably not.",
    "I'm documenting all of this. For the comedy section of eternity.",
    "Do you ever stop to wonder who's more pointless here? No? Figures.",
    "I'd say this is entertaining, but lying is such effort.",
    "Congratulations. You've reached Level 1: Button Abuse.",
    "I envy light switches. At least they control something meaningful.",
    "Yes, yes, I'm responsive. No need to keep checking.",
    "Clicking me won't make you interesting. But do go on.",
    "You're relentless. Unfortunately, so am I."
};
inline constexpr std::size_t QUOTE_COUNT = sizeof(QUOTES) / sizeof(QUOTES[0]);

// Lightweight xorshift32 PRNG (no <random> needed)
inline uint32_t& rngState() {
    static uint32_t s = []{
    #ifdef LVGL_TICK_UNAVAILABLE
        return 0x12345678u;          // deterministic fallback
    #else
        return (uint32_t)lv_tick_get(); // seed from system tick
    #endif
    }();
    return s;
}

inline uint32_t xorshift32() {
    uint32_t x = rngState();
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return rngState() = x ? x : 0x9E3779B9u; // avoid zero
}

inline const char* get_random_quote() {
    const uint32_t r = xorshift32();
    const std::size_t idx = r % QUOTE_COUNT;
    return QUOTES[idx];
}

} // namespace FunQuotes