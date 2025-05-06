
#include <time.h>

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
    int N = init_size();
    std::vector<Boid> boids(N);

    sf::RenderWindow window(sf::VideoMode({600, 600}), "My window",
                            sf::Style::Default, sf::State::Windowed);

    std::vector<sf::ConvexShape> triangles(N);

    sf::RectangleShape rectangle({400.0f, 400.0f});
    rectangle.setPosition({100.0f, 100.0f});
    rectangle.setOutlineColor(sf::Color::Black);
    rectangle.setOutlineThickness(2.0f);

    window.setFramerateLimit(60);

    add_boid(boids);
    // mean_deviation(boids);
    // mean_deviation_algo(boids);

    add_triangle(triangles);

    while (window.isOpen()) {
      while (const std::optional event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
          window.close();
        }
      }

      window.clear(sf::Color::White);
      window.draw(rectangle);

      // update_correction(boids);
      // print(triangles, boids, window);
      evaluate_correction(boids);

      auto it_b = boids.begin();

      for (std::vector<sf::ConvexShape>::iterator it = triangles.begin();
           it != triangles.end(); ++it, ++it_b) {
        init_tr((*it_b), (*it));
        (*it_b).correction();
        (*it_b).limit();
        (*it_b).vel_max();
        window.draw(*it);
      }

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