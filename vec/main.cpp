

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
// #include <execution>
#include <iostream>
#include <random>

#include "boids.hpp"

int main() {
  try {
    bob::Par parametres = bob::init_parametres();
    std::vector<bob::Boid> boids(parametres.N);

    sf::RenderWindow window(sf::VideoMode(bob::MAX_POS, bob::MAX_POS),
                            "My window");
    window.setFramerateLimit(60);

    std::vector<sf::CircleShape> circles_pred;
    std::vector<bob::Predator> predators;
    add_boid(boids);

    std::vector<sf::CircleShape> circles_boid(parametres.N);
    bob::add_circle(circles_boid);

    while (window.isOpen()) {
      sf::Event event;

      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
        if (event.type == sf::Event::MouseButtonPressed) {
          float x = event.mouseButton.x;
          float y = event.mouseButton.y;
          if (predators.size() < bob::MAX_PRED) {
            bob::Predator p(bob::Vec{x, y});
            predators.push_back(p);

            sf::CircleShape c = bob::crt_pred(x, y);
            circles_pred.push_back(c);
          }
        }
        if (event.type == sf::Event::KeyPressed &&
            event.key.code ==
                sf::Keyboard::Space) {  // condizione scritta da chat gpt
          bob::statistics::print_statistics(boids);
        }
      }
      assert(predators.size() <= bob::MAX_PRED);

      window.clear(sf::Color::White);

      evaluate_pred_correction(predators, boids);
      evaluate_corr_fuga(boids, predators);
      evaluate_correction(boids, parametres);

      update_correction(circles_pred, circles_boid, predators, boids, window);
      erase_boid(boids, predators, circles_boid);

      window.display();
    }

  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}
