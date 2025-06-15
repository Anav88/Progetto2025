#include <cassert>
#include <iostream>
#include <vector>

#include "boids.hpp"

int main() {
  try {
    bob::Par parametres = bob::init_parametres();
    std::vector<bob::Boid> boids(parametres.size);

    sf::RenderWindow window(sf::VideoMode(bob::MAX_POS, bob::MAX_POS),
                            "My window");
    window.setFramerateLimit(60);

    std::vector<sf::CircleShape> circles_pred;
    std::vector<bob::Predator> predators;
    bob::add_boid(boids);

    std::vector<sf::CircleShape> circles_boid(parametres.size);
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
            bob::Predator p(bob::Vec2f{x, y});
            predators.push_back(p);

            sf::CircleShape c = bob::create_pred(x, y);
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

      bob::evaluate_pred_correction(predators, boids);
      bob::evaluate_boid_correction(boids, predators, parametres);

      bob::erase_boid(boids, predators, circles_boid);
      bob::update_pred(circles_pred, predators, window);
      bob::update_boid(circles_boid, boids, window);

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
